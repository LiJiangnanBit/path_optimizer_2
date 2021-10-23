//
// Created by ljn on 20-3-10.
//

#include "solver/base_solver.hpp"
#include "data_struct/reference_path.hpp"
#include "data_struct/data_struct.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/vehicle_state_frenet.hpp"
#include "tools/tools.hpp"

namespace PathOptimizationNS {

BaseSolver::BaseSolver(std::shared_ptr<ReferencePath> reference_path,
                       std::shared_ptr<VehicleState> vehicle_state,
                       int iter_num,
                       bool enable_hard_constraint) :
    iter_num_(iter_num),
    enable_hard_constraint_(enable_hard_constraint),
    n_(reference_path->getSize()),
    reference_path_(reference_path),
    vehicle_state_(vehicle_state) {
    state_size_ = 3 * n_;
    control_size_ = n_ - 1;
    slack_size_ = 2 * n_;
    vars_size_ = state_size_ + control_size_ + slack_size_;
    cons_size_ = 6 * n_ + 2 + 2 * n_ * static_cast<int>(enable_hard_constraint);
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

bool BaseSolver::solve(std::vector<PathOptimizationNS::State> *optimized_path) {
    const auto &ref_states = reference_path_->getReferenceStates();
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(vars_size_);
    solver_.data()->setNumberOfConstraints(cons_size_);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(vars_size_);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    // Set Hessian matrix.
    setCost(&hessian);
    // Set state transition matrix, constraint matrix and bound vector.
    setConstraints(
        &linearMatrix,
        &lowerBound,
        &upperBound);
    // Input to solver.
    if (!solver_.data()->setHessianMatrix(hessian)) return false;
    if (!solver_.data()->setGradient(gradient)) return false;
    if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver_.data()->setLowerBound(lowerBound)) return false;
    if (!solver_.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver_.initSolver()) return false;
    if (!solver_.solve()) return false;
    const auto &QPSolution = solver_.getSolution();
    getOptimizedPath(QPSolution, optimized_path);
    return true;
}

void BaseSolver::setCost(Eigen::SparseMatrix<double> *matrix_h) const {
    Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(vars_size_, vars_size_, 0)};
    const auto weight_l = 0.1;
    const auto weight_kappa = 20.0;
    const auto weight_dkappa = 100.0;
    const auto weight_slack = 10;// 1000.0 - 200 * iter_num_;
    for (size_t i = 0; i < n_; ++i) {
        hessian(3 * i, 3 * i) += weight_l;
        hessian(3 * i + 2, 3 * i + 2) += weight_kappa;
        hessian(state_size_ + control_size_ + 2 * i, state_size_ + control_size_ + 2 * i)
            += weight_slack;
        hessian(state_size_ + control_size_ + 2 * i + 1, state_size_ + control_size_ + 2 * i + 1)
            += weight_slack;
        if (i != n_ - 1) {
            hessian(state_size_ + i, state_size_ + i) += weight_dkappa;
        }
    }
    *matrix_h = hessian.sparseView();
}

void BaseSolver::setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
                                Eigen::VectorXd *lower_bound,
                                Eigen::VectorXd *upper_bound) const {
    const auto &ref_states = reference_path_->getReferenceStates();
    const auto trans_idx = 0;
    const auto kappa_idx = trans_idx + 3 * n_;
    const auto collision_idx = kappa_idx + n_;
    const auto end_state_idx = collision_idx + 2 * n_;
    const auto hard_constraint_idx = end_state_idx + 2;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(cons_size_, vars_size_);
    // Set transition part. Ax + Bu + C = 0.
    for (size_t i = 0; i != state_size_; ++i) {
        cons(i, i) = -1;
    }
    Eigen::Matrix3d a(Eigen::Matrix3d::Zero());
    a(0, 1) = 1;
    a(1, 2) = 1;
    Eigen::Matrix<double, 3, 1> b(Eigen::MatrixXd::Constant(3, 1, 0));
    b(2, 0) = 1;
    std::vector<Eigen::MatrixXd> c_list;
    for (size_t i = 0; i != n_ - 1; ++i) {
        const auto ref_k = ref_states[i].k;
        const auto ds = ref_states[i + 1].s - ref_states[i].s;
        const auto ref_kp = (ref_states[i + 1].k - ref_k) / ds;
        a(1, 0) = -pow(ref_k, 2);
        auto A = a * ds + Eigen::Matrix3d::Identity();
        auto B = b * ds;
        cons.block(3 * (i + 1), 3 * i, 3, 3) = A;
        cons.block(3 * (i + 1), state_size_ + i, 3, 1) = B;
        Eigen::Matrix<double, 3, 1> c, ref_state;
        c << 0, 0, ref_kp;
        ref_state << 0, 0, ref_k;
        c_list.emplace_back(ds * (c - a * ref_state - b * ref_kp));
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
        cons.block(collision_idx + 2 * i, 3 * i, 2, 2) = collision_coeff;
        cons.block(collision_idx + 2 * i, state_size_ + control_size_ + 2 * i, 2, 2) = slack_coeff;
    }
    // End state.
    cons(end_state_idx, state_size_ - 3) = 1; // end l
    cons(end_state_idx + 1, state_size_ - 2) = 1; // end ephi
    // Hard constraint.
    if (enable_hard_constraint_) {
        for (size_t i = 0; i < n_; ++i) {
            cons.block(hard_constraint_idx + 2 * i, 3 * i, 2, 2) = collision_coeff;
        }
    }
    *matrix_constraints = cons.sparseView();

    // Set bounds.
    // Transition.
    *lower_bound = Eigen::MatrixXd::Zero(cons_size_, 1);
    *upper_bound = Eigen::MatrixXd::Zero(cons_size_, 1);
    Eigen::Matrix<double, 3, 1> x0;
    const auto init_error = vehicle_state_->getInitError();
    x0 << init_error[0], init_error[1], vehicle_state_->getStartState().k;
    lower_bound->block(0, 0, 3, 1) = -x0;
    upper_bound->block(0, 0, 3, 1) = -x0;
    for (size_t i = 0; i < n_ - 1; ++i) {
        lower_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
        upper_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
    }
    // Kappa.
    for (size_t i = 0; i < n_; ++i) {
        (*lower_bound)(kappa_idx + i) = -tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
        (*upper_bound)(kappa_idx + i) = tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
    }
    // Collision.
    const auto &bounds = reference_path_->getBounds();
    for (size_t i = 0; i < n_; ++i) {
        const auto front_bounds = getSoftBounds(bounds[i].front.lb, bounds[i].front.ub, FLAGS_expected_safety_margin);
        const auto rear_bounds = getSoftBounds(bounds[i].rear.lb, bounds[i].rear.ub, FLAGS_expected_safety_margin);
        (*lower_bound)(collision_idx + 2 * i, 0) = front_bounds.first;
        (*upper_bound)(collision_idx + 2 * i, 0) = front_bounds.second;
        (*lower_bound)(collision_idx + 2 * i + 1, 0) = rear_bounds.first;
        (*upper_bound)(collision_idx + 2 * i + 1, 0) = rear_bounds.second;
    }
    // End state.
    (*lower_bound)(end_state_idx) = -1.0; //-OsqpEigen::INFTY;
    (*upper_bound)(end_state_idx) = 1.0; //OsqpEigen::INFTY;
    (*lower_bound)(end_state_idx + 1) = -OsqpEigen::INFTY;
    (*upper_bound)(end_state_idx + 1) = OsqpEigen::INFTY;
    if (FLAGS_constraint_end_heading && reference_path_->isBlocked() != nullptr) {
        double end_psi = constrainAngle(vehicle_state_->getTargetState().heading - ref_states.back().heading);
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(end_state_idx + 1) = end_psi - 0.087; // 5 degree.
            (*upper_bound)(end_state_idx + 1) = end_psi + 0.087;
        }
    }
    if (enable_hard_constraint_) {
        for (size_t i = 0; i < n_; ++i) {
            (*lower_bound)(hard_constraint_idx + 2 * i, 0) = bounds[i].front.lb;
            (*upper_bound)(hard_constraint_idx + 2 * i, 0) = bounds[i].front.ub;
            (*lower_bound)(hard_constraint_idx + 2 * i + 1, 0) = bounds[i].rear.lb;
            (*upper_bound)(hard_constraint_idx + 2 * i + 1, 0) = bounds[i].rear.ub;
        }
    }
}

void BaseSolver::getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<PathOptimizationNS::State> *optimized_path) const {
    CHECK_EQ(optimization_result.size(), vars_size_);
    optimized_path->clear();
    const auto &ref_states = reference_path_->getReferenceStates();
    double tmp_s = 0;
    for (size_t i = 0; i != n_; ++i) {
        double angle = ref_states[i].heading;
        double new_angle = constrainAngle(angle + M_PI_2);
        double tmp_x = ref_states[i].x + optimization_result(3 * i) * cos(new_angle);
        double tmp_y = ref_states[i].y + optimization_result(3 * i) * sin(new_angle);
        double k = optimization_result(3 * i + 2);
        if (i != 0) {
            tmp_s += sqrt(pow(tmp_x - optimized_path->back().x, 2) + pow(tmp_y - optimized_path->back().y, 2));
        }
        optimized_path->emplace_back(tmp_x, tmp_y, constrainAngle(angle + optimization_result(3 * i + 1)), k, tmp_s);
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