//
// Created by ljn on 20-3-10.
//

#include "solver/base_solver.hpp"
#include "data_struct/reference_path.hpp"
#include "data_struct/data_struct.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/vehicle_state_frenet.hpp"
#include "tools/tools.hpp"
#include "tools/time_recorder.h"

namespace PathOptimizationNS {

BaseSolver::BaseSolver(const ReferencePath &reference_path,
               const VehicleState &vehicle_state,
               const std::vector<SlState> &input_path) :
    n_(input_path.size()),
    reference_path_(reference_path),
    vehicle_state_(vehicle_state),
    input_path_(input_path) {
    state_size_ = 3 * n_;
    control_size_ = n_ - 1;
    precise_planning_size_ = input_path.size();
    if (FLAGS_rough_constraints_far_away) {
        const auto precise_planning_iter = std::lower_bound(
            input_path.begin(),
            input_path.end(),
            FLAGS_precise_planning_length,
            [](const SlState& state, double s){
            return state.s < s;
            });
        precise_planning_size_ = std::distance(input_path.begin(), precise_planning_iter);
    }
    slack_size_ = precise_planning_size_ + n_;
    vars_size_ = state_size_ + control_size_ + slack_size_;
    cons_size_ = 4 * n_ + precise_planning_size_ + n_ + 2;
    LOG(INFO) << "Ref length " << reference_path_.getLength() << ", precise_planning_size_ " << precise_planning_size_;
}

std::unique_ptr<BaseSolver> BaseSolver::create(std::string &type,
                                               std::shared_ptr<ReferencePath> reference_path,
                                               std::shared_ptr<VehicleState> vehicle_state) {
    // if (type == "K") {
    //     return std::unique_ptr<OsqpSolver>(new SolverKAsInput(reference_path, vehicle_state, horizon));
    // } else if (type == "KP") {
    //     return std::unique_ptr<OsqpSolver>(new SolverKpAsInput(reference_path, vehicle_state, horizon));
    // } else if (type == "KPC") {
    //     return std::unique_ptr<OsqpSolver>(new SolverKpAsInputConstrained(reference_path, vehicle_state, horizon));
    // } else {
    //     LOG(ERROR) << "No such solver!";
    //     return nullptr;
    // }
}

bool BaseSolver::solve(std::vector<SlState> *optimized_path) {
    TimeRecorder time_recorder("First Solver");
    time_recorder.recordTime("Set cost");
    solver_.settings()->setVerbosity(true);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setAbsoluteTolerance(2e-3);
    solver_.settings()->setRelativeTolerance(2e-3);
    solver_.data()->setNumberOfVariables(vars_size_);
    solver_.data()->setNumberOfConstraints(cons_size_);
    // Allocate QP problem matrices and vectors.
    gradient_ = Eigen::VectorXd::Zero(vars_size_);
    // Eigen::SparseMatrix<double> linearMatrix;
    // Eigen::VectorXd lowerBound;
    // Eigen::VectorXd upperBound;
    // Set Hessian matrix.
    setCost(&hessian_);
    time_recorder.recordTime("Set constraints");
    // Set state transition matrix, constraint matrix and bound vector.
    setConstraints(
        &linear_matrix_,
        &lower_bound_,
        &upper_bound_);
    time_recorder.recordTime("Set solver");
    // Input to solver.
    if (!solver_.data()->setHessianMatrix(hessian_)) return false;
    if (!solver_.data()->setGradient(gradient_)) return false;
    if (!solver_.data()->setLinearConstraintsMatrix(linear_matrix_)) return false;
    if (!solver_.data()->setLowerBound(lower_bound_)) return false;
    if (!solver_.data()->setUpperBound(upper_bound_)) return false;
    // Solve.
    time_recorder.recordTime("OSQP Solve");
    if (!solver_.initSolver()) return false;
    if (!solver_.solve()) return false;
    const auto &QPSolution = solver_.getSolution();
    time_recorder.recordTime("Retrive path");
    getOptimizedPath(QPSolution, optimized_path);
    time_recorder.recordTime("end");
    time_recorder.printTime();
    return true;
}

bool BaseSolver::updateProblemFormulationAndSolve(const std::vector<SlState> &input_path, std::vector<SlState> *optimized_path) {
    TimeRecorder time_recorder("Second Solver");
    time_recorder.recordTime("Set constraints");
    input_path_ = input_path;
    // Set state transition matrix, constraint matrix and bound vector.
    setConstraints(
        &linear_matrix_,
        &lower_bound_,
        &upper_bound_);
    if (!solver_.updateBounds(lower_bound_, upper_bound_)) return false;
    if (!solver_.updateLinearConstraintsMatrix(linear_matrix_)) return false;
    // Solve.
    time_recorder.recordTime("OSQP Solve");
    if (!solver_.solve()) return false;
    time_recorder.recordTime("Retrive path");
    const auto &QPSolution = solver_.getSolution();
    getOptimizedPath(QPSolution, optimized_path);
    time_recorder.recordTime("end");
    time_recorder.printTime();
    return true;
}

void BaseSolver::setCost(Eigen::SparseMatrix<double> *matrix_h) const {
    TimeRecorder time_recorder("Set Cost");
    time_recorder.recordTime("set heassian");
    Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(vars_size_, vars_size_, 0)};
    const double weight_l = 0.0;
    const double weight_kappa = 20.0;
    const double weight_dkappa = 100.0;
    const double weight_slack = 10;// 1000.0 - 200 * iter_num_;
    for (size_t i = 0; i < n_; ++i) {
        hessian(3 * i, 3 * i) += weight_l;
        hessian(3 * i + 2, 3 * i + 2) += weight_kappa;
        if (i < precise_planning_size_) {
            hessian(state_size_ + control_size_ + 2 * i, state_size_ + control_size_ + 2 * i)
                += weight_slack;
            hessian(state_size_ + control_size_ + 2 * i + 1, state_size_ + control_size_ + 2 * i + 1)
                += weight_slack;
        } else {
            size_t local_index = i - precise_planning_size_;
            size_t slack_var_index = state_size_ + control_size_ + 2 * precise_planning_size_ + local_index;
            hessian(slack_var_index, slack_var_index) += weight_slack;
        }
        if (i != n_ - 1) {
            hessian(state_size_ + i, state_size_ + i) += weight_dkappa;
        }
    }
    time_recorder.recordTime("return matrix");
    *matrix_h = hessian.sparseView();
    time_recorder.recordTime("end");
    time_recorder.printTime();
}

void BaseSolver::setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
                                Eigen::VectorXd *lower_bound,
                                Eigen::VectorXd *upper_bound) const {
    const auto &ref_states = reference_path_.getReferenceStates();
    const auto trans_idx = 0;
    const auto kappa_idx = trans_idx + 3 * n_;
    const auto precise_collision_idx = kappa_idx + n_;
    const auto rough_collision_idx = precise_collision_idx + 2 * precise_planning_size_;
    const auto end_state_idx = rough_collision_idx + n_ - precise_planning_size_;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(cons_size_, vars_size_);
    // Set transition part. Ax_i + Bu_i + C = x_(i+1).
    for (size_t i = 0; i != state_size_; ++i) {
        cons(i, i) = -1;
    }
    std::vector<Eigen::MatrixXd> c_list;
    for (size_t i = 0; i < n_ - 1; ++i) {
        const auto &x = input_path_.at(i);
        const auto &x_next = input_path_.at(i+1);
        Eigen::Matrix3d df_x(Eigen::Matrix3d::Zero());
        df_x << -x.k * tan(x.d_heading), (1-x.k*x.l) / pow(cos(x.d_heading), 2), 0,
                -x.k * x.k / cos(x.d_heading), (1-x.k*x.l) * x.k * tan(x.d_heading) / cos(x.d_heading), (1-x.k*x.l) / cos(x.d_heading),
                0, 0, 0;
        Eigen::Matrix<double, 3, 1> df_u;
        df_u << 0, 0, 1;
        const auto ds = ref_states[i + 1].s - ref_states[i].s;
        const auto A = ds * df_x + Eigen::Matrix3d::Identity();
        const auto B = ds * df_u;
        cons.block(3 * (i + 1), 3 * i, 3, 3) = A;
        cons.block(3 * (i + 1), state_size_ + i, 3, 1) = B;
        Eigen::Matrix<double, 3, 1> f_x_u;
        const double u_input = (x_next.k - x.k) / ds;
        f_x_u << (1-x.k*x.l) * tan(x.d_heading),
                (1-x.k*x.l)*x.k / cos(x.d_heading) - ref_states[i].k,
                u_input; // TODO: use dk directly.
        Eigen::Matrix<double, 3, 1> x_vec;
        x_vec << x.l, x.d_heading, x.k;
        c_list.emplace_back(ds *(f_x_u - df_x * x_vec - df_u * u_input));
    }
    // Kappa.
    for (size_t i = 0; i < n_; ++i) {
        cons(kappa_idx + i, 3 * i + 2) = 1;
    }
    // Collision.
    Eigen::Matrix<double, 2, 2> collision_coeff;
    collision_coeff << 1, FLAGS_front_length, 1, FLAGS_rear_length;
    Eigen::Matrix<double, 2, 2> slack_coeff;
    slack_coeff << 1, 0, 0, 1;
    for (size_t i = 0; i < n_; ++i) {
        if (i < precise_planning_size_) {
            cons.block(precise_collision_idx + 2 * i, 3 * i, 2, 2) = collision_coeff;
            cons.block(precise_collision_idx + 2 * i, state_size_ + control_size_ + 2 * i, 2, 2) = slack_coeff;
        } else {
            size_t local_index = i - precise_planning_size_;
            cons(rough_collision_idx + local_index, 3 * i) = 1;
            cons(rough_collision_idx + local_index, state_size_ + control_size_ + 2 * precise_planning_size_ + local_index) = 1;
        }
    }
    // End state.
    cons(end_state_idx, state_size_ - 3) = 1; // end l
    cons(end_state_idx + 1, state_size_ - 2) = 1; // end ephi
    *matrix_constraints = cons.sparseView();

    // Set bounds.
    // Transition.
    *lower_bound = Eigen::MatrixXd::Zero(cons_size_, 1);
    *upper_bound = Eigen::MatrixXd::Zero(cons_size_, 1);
    Eigen::Matrix<double, 3, 1> x0;
    const auto init_error = vehicle_state_.getInitError();
    x0 << init_error[0], init_error[1], vehicle_state_.getStartState().k;
    lower_bound->block(0, 0, 3, 1) = -x0;
    upper_bound->block(0, 0, 3, 1) = -x0;
    for (size_t i = 0; i < n_ - 1; ++i) {
        lower_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
        upper_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
    }
    // Kappa.
    const double kappa_limit = tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
    LOG(INFO) << "kappa limit " << kappa_limit;
    for (size_t i = 0; i < n_; ++i) {
        (*lower_bound)(kappa_idx + i) = -kappa_limit;
        (*upper_bound)(kappa_idx + i) = kappa_limit;
    }
    // Collision.
    const auto &bounds = reference_path_.getBounds();
    for (size_t i = 0; i < n_; ++i) {
        if (i < precise_planning_size_) {
            const auto front_bounds = getSoftBounds(bounds[i].front.lb, bounds[i].front.ub, FLAGS_expected_safety_margin);
            const auto rear_bounds = getSoftBounds(bounds[i].rear.lb, bounds[i].rear.ub, FLAGS_expected_safety_margin);
            (*lower_bound)(precise_collision_idx + 2 * i, 0) = front_bounds.first;
            (*upper_bound)(precise_collision_idx + 2 * i, 0) = front_bounds.second;
            (*lower_bound)(precise_collision_idx + 2 * i + 1, 0) = rear_bounds.first;
            (*upper_bound)(precise_collision_idx + 2 * i + 1, 0) = rear_bounds.second;
        } else {
            const auto center_bounds = getSoftBounds(bounds[i].center.lb, bounds[i].center.ub, FLAGS_expected_safety_margin);
            size_t local_index = i - precise_planning_size_;
            (*lower_bound)(rough_collision_idx + local_index, 0) = center_bounds.first;
            (*upper_bound)(rough_collision_idx + local_index, 0) = center_bounds.second;
        }
    }
    // End state.
    (*lower_bound)(end_state_idx) = -1.0; //-OsqpEigen::INFTY;
    (*upper_bound)(end_state_idx) = 1.0; //OsqpEigen::INFTY;
    (*lower_bound)(end_state_idx + 1) = -OsqpEigen::INFTY;
    (*upper_bound)(end_state_idx + 1) = OsqpEigen::INFTY;
    if (FLAGS_constraint_end_heading && reference_path_.isBlocked() == nullptr) {
        double end_psi = constrainAngle(vehicle_state_.getTargetState().heading - ref_states.back().heading);
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(end_state_idx + 1) = end_psi - 0.087; // 5 degree.
            (*upper_bound)(end_state_idx + 1) = end_psi + 0.087;
        }
    }
}

void BaseSolver::getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<SlState> *optimized_path) const {
    CHECK_EQ(optimization_result.size(), vars_size_);
    optimized_path->clear();
    const auto &ref_states = reference_path_.getReferenceStates();
    double tmp_s = 0;
    for (size_t i = 0; i != n_; ++i) {
        SlState result_pt;
        double angle = ref_states[i].heading;
        result_pt.heading = constrainAngle(angle + optimization_result(3 * i + 1));
        result_pt.d_heading = optimization_result(3 * i + 1);
        result_pt.l = optimization_result(3 * i);
        double new_angle = constrainAngle(angle + M_PI_2);
        result_pt.x = ref_states[i].x + optimization_result(3 * i) * cos(new_angle);
        result_pt.y = ref_states[i].y + optimization_result(3 * i) * sin(new_angle);
        result_pt.k = optimization_result(3 * i + 2);
        if (i < n_ - 1) {
            result_pt.d_k = optimization_result(3 * n_ + i);
        }
        if (i != 0) {
            tmp_s += sqrt(pow(result_pt.x - optimized_path->back().x, 2) + pow(result_pt.y - optimized_path->back().y, 2));
        }
        optimized_path->push_back(result_pt);
        // LOG(INFO) << "idx " << i << " l diff with input " << result_pt.l - input_path_.at(i).l << ", input l " << input_path_.at(i).l << ", opt l" << result_pt.l;
    }
}

std::pair<double, double> BaseSolver::getSoftBounds(double lb, double ub, double safety_margin) const {
    const auto clearance = ub - lb;
    static const auto min_clearance = 0.1;
    auto remain_clearance = std::max(min_clearance, clearance - 2 * safety_margin);
    auto shrink = std::max(0.0, (clearance - remain_clearance) / 2.0);
    return std::make_pair(lb + shrink, ub - shrink);
}

}