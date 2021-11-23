//
// Created by ljn on 20-5-4.
//
#include <tinyspline_ros/tinysplinecpp.h>
#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include "tools/Map.hpp"
#include "reference_path_smoother/tension_smoother_2.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/data_struct.hpp"

namespace PathOptimizationNS {
TensionSmoother2::TensionSmoother2(const std::vector<PathOptimizationNS::State> &input_points,
                                   const PathOptimizationNS::State &start_state,
                                   const PathOptimizationNS::Map &grid_map) :
    TensionSmoother(input_points, start_state, grid_map) {}

bool TensionSmoother2::osqpSmooth(const std::vector<double> &x_list,
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
    solver.data()->setNumberOfVariables(4 * point_num - 1);
    solver.data()->setNumberOfConstraints(3 * (point_num - 1) + 2);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setHessianMatrix(point_num, &hessian);
    setGradient(x_list, y_list, &gradient);
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

void TensionSmoother2::setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    const size_t theta_start_index = y_start_index + size;
    const size_t k_start_index = theta_start_index + size;
    const size_t matrix_size = 4 * size - 1;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // Deviation and curvature.
    for (int i = 0; i != size; ++i) {
        hessian(x_start_index + i, x_start_index + i) = hessian(y_start_index + i, y_start_index + i)
            = FLAGS_tension_2_deviation_weight * 2;
        if (i != size - 1) hessian(k_start_index + i, k_start_index + i) = FLAGS_tension_2_curvature_weight * 2;
    }
    // Curvature change.
    Eigen::Vector2d coeff_vec{1, -1};
    Eigen::Matrix2d coeff = coeff_vec * coeff_vec.transpose();
    for (int i = 0; i != size - 2; ++i) {
        hessian.block(k_start_index + i, k_start_index + i, 2, 2) += 2 * FLAGS_tension_2_curvature_rate_weight * coeff;
    }
    *matrix_h = hessian.sparseView();
}

void TensionSmoother2::setConstraintMatrix(const std::vector<double> &x_list,
                                           const std::vector<double> &y_list,
                                           const std::vector<double> &angle_list,
                                           const std::vector<double> &k_list,
                                           const std::vector<double> &s_list,
                                           Eigen::SparseMatrix<double> *matrix_constraints,
                                           Eigen::VectorXd *lower_bound,
                                           Eigen::VectorXd *upper_bound) const {
    const size_t size = x_list.size();
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    const size_t theta_start_index = y_start_index + size;
    const size_t k_start_index = theta_start_index + size;
    const size_t cons_x_update_start_index = 0;
    const size_t cons_y_update_start_index = cons_x_update_start_index + size - 1;
    const size_t cons_theta_update_start_index = cons_y_update_start_index + size - 1;
    const size_t cons_x_index = cons_theta_update_start_index + size - 1;
    const size_t cons_y_index = cons_x_index + 1;

    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 4 * size - 1);
    *lower_bound = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 1);
    // Cons.
    for (int i = 0; i != size - 1; ++i) {
        const double ds = s_list[i + 1] - s_list[i];
        cons(cons_x_update_start_index + i, x_start_index + i + 1) =
        cons(cons_y_update_start_index + i, y_start_index + i + 1)
            = cons(cons_theta_update_start_index + i, theta_start_index + i + 1) = 1;
        cons(cons_x_update_start_index + i, x_start_index + i) = cons(cons_y_update_start_index + i, y_start_index + i)
            = cons(cons_theta_update_start_index + i, theta_start_index + i) = -1;
        cons(cons_x_update_start_index + i, theta_start_index + i) = ds * sin(angle_list[i]);
        cons(cons_y_update_start_index + i, theta_start_index + i) = -ds * cos(angle_list[i]);
        cons(cons_theta_update_start_index + i, k_start_index + i) = -ds;
    }
    cons(cons_x_index, x_start_index) = cons(cons_y_index, y_start_index) = 1;
    *matrix_constraints = cons.sparseView();
    // Bounds.
    for (int i = 0; i != size - 1; ++i) {
        const double ds = s_list[i + 1] - s_list[i];
        (*lower_bound)(cons_x_update_start_index + i) = (*upper_bound)(cons_x_update_start_index + i) =
            ds * cos(angle_list[i]);
        (*lower_bound)(cons_y_update_start_index + i) = (*upper_bound)(cons_y_update_start_index + i) =
            ds * sin(angle_list[i]);
        (*lower_bound)(cons_theta_update_start_index + i) = (*upper_bound)(cons_theta_update_start_index + i) =
            -ds * k_list[i];

    }
    (*lower_bound)(cons_x_index) = (*upper_bound)(cons_x_index) = x_list[0];
    (*lower_bound)(cons_y_index) = (*upper_bound)(cons_y_index) = y_list[0];
}

void TensionSmoother2::setGradient(const std::vector<double> &x_list,
                                   const std::vector<double> &y_list,
                                   Eigen::VectorXd *gradient) {
    const auto size = x_list.size();
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    *gradient = Eigen::VectorXd::Constant(4 * size - 1, 0);
    for (int i = 0; i != size; ++i) {
        (*gradient)(x_start_index + i) = -2 * FLAGS_tension_2_deviation_weight * x_list[i];
        (*gradient)(y_start_index + i) = -2 * FLAGS_tension_2_deviation_weight * y_list[i];
    }
}
}