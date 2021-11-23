//
// Created by ljn on 20-4-14.
//
#include <tinyspline_ros/tinysplinecpp.h>
#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include "tools/Map.hpp"
#include "reference_path_smoother/tension_smoother.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/data_struct.hpp"

namespace PathOptimizationNS {
TensionSmoother::TensionSmoother(const std::vector<PathOptimizationNS::State> &input_points,
                                 const PathOptimizationNS::State &start_state,
                                 const PathOptimizationNS::Map &grid_map) :
    ReferencePathSmoother(input_points, start_state, grid_map) {}

bool TensionSmoother::smooth(std::shared_ptr<PathOptimizationNS::ReferencePath> reference_path) {
    std::vector<double> x_list, y_list, s_list, angle_list, k_list;
    if (!segmentRawReference(&x_list, &y_list, &s_list, &angle_list, &k_list)) return false;
    std::vector<double> result_x_list, result_y_list, result_s_list;
    bool solver_ok = osqpSmooth(x_list,
                               y_list,
                               angle_list,
                               k_list,
                               s_list,
                               &result_x_list,
                               &result_y_list,
                               &result_s_list);
    if (!solver_ok) {
        LOG(ERROR) << "Tension smoother failed!";
        return false;
    }
    tk::spline x_spline, y_spline;
    x_spline.set_points(result_s_list, result_x_list);
    y_spline.set_points(result_s_list, result_y_list);

    double max_s_result = result_s_list.back() + 3;
    reference_path->setSpline(x_spline, y_spline, max_s_result);

    x_list_ = std::move(result_x_list);
    y_list_ = std::move(result_y_list);
    s_list_ = std::move(result_s_list);
    return true;
}

bool TensionSmoother::osqpSmooth(const std::vector<double> &x_list,
                                 const std::vector<double> &y_list,
                                 const std::vector<double> &angle_list,
                                 const std::vector<double> &k_list,
                                 const std::vector<double> &s_list,
                                 std::vector<double> *result_x_list,
                                 std::vector<double> *result_y_list,
                                 std::vector<double> *result_s_list) {
    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), angle_list.size());
    CHECK_EQ(angle_list.size(), s_list.size());
    auto point_num = x_list.size();
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(3 * point_num);
    solver.data()->setNumberOfConstraints(3 * point_num);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setHessianMatrix(point_num, &hessian);
    setConstraintMatrix(x_list, y_list, angle_list, k_list, s_list, &linearMatrix, &lowerBound, &upperBound);
    // Input to solver.
    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver.initSolver()) return false;
    if (!solver.solve()) return false;
    const auto &QPSolution{solver.getSolution()};
    // Output.
    result_s_list->clear();
    result_x_list->clear();
    result_y_list->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != point_num; ++i) {
        double tmp_x = QPSolution(i);
        double tmp_y = QPSolution(point_num + i);
        result_x_list->emplace_back(tmp_x);
        result_y_list->emplace_back(tmp_y);
        if (i != 0)
            tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                              + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}

void TensionSmoother::setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t x_start_index{0};
    const size_t y_start_index{x_start_index + size};
    const size_t d_start_index{y_start_index + size};
    const size_t matrix_size = 3 * size;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // Curvature part.
    Eigen::Matrix<double, 3, 1> dds_vec{1, -2, 1};
    Eigen::Matrix3d dds_part{dds_vec * dds_vec.transpose() * FLAGS_cartesian_curvature_weight};
    Eigen::Matrix<double, 4, 1> ddds_vec{-1, 3, -3, 1};
    Eigen::Matrix4d ddds_part{ddds_vec * ddds_vec.transpose() * FLAGS_cartesian_curvature_rate_weight};
    for (int i = 0; i != size - 2; ++i) {
        hessian.block(x_start_index + i, x_start_index + i, 3, 3) += dds_part;
        hessian.block(y_start_index + i, y_start_index + i, 3, 3) += dds_part;
        if (i != size - 3) {
            hessian.block(x_start_index + i, x_start_index + i, 4, 4) += ddds_part;
            hessian.block(y_start_index + i, y_start_index + i, 4, 4) += ddds_part;
        }
    }
    // Deviation part.
    for (int i = 0; i != size; ++i) {
        hessian(d_start_index + i, d_start_index + i) = FLAGS_cartesian_deviation_weight;
    }
    *matrix_h = hessian.sparseView();
}

void TensionSmoother::setConstraintMatrix(const std::vector<double> &x_list,
                                          const std::vector<double> &y_list,
                                          const std::vector<double> &angle_list,
                                          const std::vector<double> &k_list,
                                          const std::vector<double> &s_list,
                                          Eigen::SparseMatrix<double> *matrix_constraints,
                                          Eigen::VectorXd *lower_bound,
                                          Eigen::VectorXd *upper_bound) const {
    const size_t size{x_list.size()};
    const size_t x_start_index{0};
    const size_t y_start_index{x_start_index + size};
    const size_t d_start_index{y_start_index + size};
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * size, 3 * size);
    *lower_bound = Eigen::MatrixXd::Zero(3 * size, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * size, 1);
    for (int i = 0; i != size; ++i) {
        // x, y and d
        cons(x_start_index + i, x_start_index + i) = cons(y_start_index + i, y_start_index + i) = 1;
        double theta{angle_list[i] + M_PI_2};
        cons(x_start_index + i, d_start_index + i) = -cos(theta);
        cons(y_start_index + i, d_start_index + i) = -sin(theta);
        // d
        cons(d_start_index + i, d_start_index + i) = 1;
        // bounds
        (*lower_bound)(x_start_index + i) = x_list[i];
        (*upper_bound)(x_start_index + i) = x_list[i];
        (*lower_bound)(y_start_index + i) = y_list[i];
        (*upper_bound)(y_start_index + i) = y_list[i];
    }
    *matrix_constraints = cons.sparseView();
    // d bounds.
    (*lower_bound)(d_start_index) = 0;
    (*upper_bound)(d_start_index) = 0;
    (*lower_bound)(d_start_index + size - 1) = -0.5;
    (*upper_bound)(d_start_index + size - 1) = 0.5;
    const double default_clearance = 2;
//    const double shrink_clearance = 0;
    for (size_t i = 1; i != size - 1; ++i) {
        double x = x_list[i];
        double y = y_list[i];
        double clearance = grid_map_.getObstacleDistance(grid_map::Position(x, y));
        // Adjust clearance.
        clearance = std::min(clearance, default_clearance);
        LOG(INFO) << "id: " << i << ", " << clearance;
//            isEqual(clearance, 0) ? default_clearance :
//                   clearance > shrink_clearance ? clearance - shrink_clearance : clearance;
        (*lower_bound)(d_start_index + i) = -clearance;
        (*upper_bound)(d_start_index + i) = clearance;
    }
}

}
