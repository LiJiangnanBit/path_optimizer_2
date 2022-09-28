#include "solver/simplified_solver.hpp"
#include "data_struct/reference_path.hpp"
#include "data_struct/data_struct.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/vehicle_state_frenet.hpp"
#include "tools/tools.hpp"
#include "tools/time_recorder.h"

namespace PathOptimizationNS {

SimplifiedSolver::SimplifiedSolver(const ReferencePath &reference_path,
               const VehicleState &vehicle_state,
               const std::vector<SlState> &input_path)
               : BaseSolver(reference_path, vehicle_state, input_path) {
    state_size_ = 2 * n_;
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
    cons_size_ = 3 * n_ + precise_planning_size_ + n_ + 1;
    LOG(INFO) << "Ref length " << reference_path_.getLength() << ", precise_planning_size_ " << precise_planning_size_;
}

void SimplifiedSolver::setHessian(Eigen::SparseMatrix<double> *matrix_h) const {
    CHECK_NOTNULL(matrix_h);
    Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(vars_size_, vars_size_, 0)};
    const double weight_l = 0.0;
    const double weight_kappa = 20.0;
    const double weight_dkappa = 50.0;
    const double weight_slack = 100.0;
    Eigen::Matrix2d dkappa_coeff;
    dkappa_coeff << 1, -1, -1, 1;
    for (size_t i = 0; i < n_; ++i) {
        hessian(2*i, 2*i) += weight_l;
        if (i != n_ - 1) {
            hessian(state_size_ + i, state_size_ + i) += weight_kappa;
        }
        if (i < n_ - 2) {
            hessian.block(state_size_ + i, state_size_ + i, 2, 2) += dkappa_coeff * weight_dkappa;
        }
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
    }
    *matrix_h = hessian.sparseView();
}

void SimplifiedSolver::setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
                                Eigen::VectorXd *lower_bound,
                                Eigen::VectorXd *upper_bound) const {
    const auto &ref_states = reference_path_.getReferenceStates();
    const auto trans_idx = 0;
    const auto kappa_idx = trans_idx + 2 * n_;
    const auto precise_collision_idx = kappa_idx + n_ - 1;
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
        const auto &x_ref = ref_states[i];
        Eigen::Matrix2d df_x;;
        df_x << -x_ref.k * tan(x.d_heading), (1-x_ref.k*x.l) / pow(cos(x.d_heading), 2),
                -x_ref.k * x.k / cos(x.d_heading), (1-x_ref.k*x.l) * x.k * tan(x.d_heading) / cos(x.d_heading);
        Eigen::Matrix<double, 2, 1> df_u;
        df_u << 0, (1 - x_ref.k * x.l) / cos(x.d_heading);
        const auto ds = ref_states[i + 1].s - ref_states[i].s;
        const auto A = ds * df_x + Eigen::Matrix2d::Identity();
        const auto B = ds * df_u;
        cons.block(2 * (i + 1), 2 * i, 2, 2) = A;
        cons.block(2 * (i + 1), state_size_ + i, 2, 1) = B;
        Eigen::Matrix<double, 2, 1> f_x_u;
        f_x_u << (1-x_ref.k*x.l) * tan(x.d_heading),
                (1-x_ref.k*x.l)*x.k / cos(x.d_heading) - x_ref.k;
        Eigen::Matrix<double, 2, 1> x_vec;
        x_vec << x.l, x.d_heading;
        c_list.emplace_back(ds *(f_x_u - df_x * x_vec - df_u * x.k));
    }
    // Kappa.
    for (size_t i = 0; i < n_ - 1; ++i) {
        cons(kappa_idx + i, state_size_ + i) = 1;
    }
    // Collision.
    Eigen::Matrix<double, 2, 2> collision_coeff;
    collision_coeff << 1, FLAGS_front_length, 1, FLAGS_rear_length * 0.5;
    Eigen::Matrix<double, 2, 2> slack_coeff;
    slack_coeff << 1, 0, 0, 1;
    for (size_t i = 0; i < n_; ++i) {
        if (i < precise_planning_size_) {
            cons.block(precise_collision_idx + 2 * i, 2 * i, 2, 2) = collision_coeff;
            cons.block(precise_collision_idx + 2 * i, state_size_ + control_size_ + 2 * i, 2, 2) = slack_coeff;
        } else {
            size_t local_index = i - precise_planning_size_;
            cons(rough_collision_idx + local_index, 2 * i) = 1;
            cons(rough_collision_idx + local_index, state_size_ + control_size_ + 2 * precise_planning_size_ + local_index) = 1;
        }
    }
    // End state.
    cons(end_state_idx, state_size_ - 2) = 1; // end l
    cons(end_state_idx + 1, state_size_ - 1) = 1; // end ephi
    *matrix_constraints = cons.sparseView();

    // Set bounds.
    // Transition.
    *lower_bound = Eigen::MatrixXd::Zero(cons_size_, 1);
    *upper_bound = Eigen::MatrixXd::Zero(cons_size_, 1);
    Eigen::Matrix<double, 2, 1> x0;
    const auto init_error = vehicle_state_.getInitError();
    x0 << init_error[0], init_error[1];
    lower_bound->block(0, 0, 2, 1) = -x0;
    upper_bound->block(0, 0, 2, 1) = -x0;
    for (size_t i = 0; i < n_ - 1; ++i) {
        lower_bound->block(2 * (i + 1), 0, 2, 1) = -c_list[i];
        upper_bound->block(2 * (i + 1), 0, 2, 1) = -c_list[i];
    }
    // Kappa.
    const double kappa_limit = tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
    LOG(INFO) << "kappa limit " << kappa_limit;
    for (size_t i = 0; i < n_ - 1; ++i) {
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
    if (FLAGS_constraint_end_heading) {
        double end_psi = constrainAngle(vehicle_state_.getTargetState().heading - ref_states.back().heading);
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(end_state_idx + 1) = end_psi - M_PI / 6.0; // 5 degree.
            (*upper_bound)(end_state_idx + 1) = end_psi + M_PI / 6.0;
        }
    }
}

void SimplifiedSolver::getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<SlState> *optimized_path) const {
    CHECK_EQ(optimization_result.size(), vars_size_);
    optimized_path->clear();
    const auto &ref_states = reference_path_.getReferenceStates();
    double tmp_s = 0;
    for (size_t i = 0; i != n_; ++i) {
        SlState result_pt;
        double angle = ref_states[i].heading;
        result_pt.heading = constrainAngle(angle + optimization_result(2 * i + 1));
        result_pt.d_heading = optimization_result(2 * i + 1);
        result_pt.l = optimization_result(2 * i);
        double new_angle = constrainAngle(angle + M_PI_2);
        result_pt.x = ref_states[i].x + optimization_result(2 * i) * cos(new_angle);
        result_pt.y = ref_states[i].y + optimization_result(2 * i) * sin(new_angle);
        if (i != n_ - 1) {
            result_pt.k = optimization_result(state_size_ + i);
        } else {
            result_pt.k = optimization_result(state_size_ + control_size_ - 1);
        }
        if (i != 0) {
            tmp_s += sqrt(pow(result_pt.x - optimized_path->back().x, 2) + pow(result_pt.y - optimized_path->back().y, 2));
        }
        result_pt.s = tmp_s;
        optimized_path->push_back(result_pt);
        // LOG(INFO) << "result values: " << optimization_result(3 * i) << ", " << optimization_result(3 * i + 1) << ", " << optimization_result(3 * i + 2) << ", " << optimization_result(3 * n_ + i);
        // LOG(INFO) << "idx " << i << " l diff with input " << result_pt.l - input_path_.at(i).l << ", input l " << input_path_.at(i).l << ", opt l" << result_pt.l;
        // LOG(INFO) << "idx " << i << ", lambda " << optimization_result(state_size_ + control_size_ + 2*i) << ", " << optimization_result(state_size_ + control_size_ + 2*i+1);
        // if (fabs(optimization_result(state_size_ + control_size_ + 2*i+1)) > 0.02) {
        //     LOG(INFO) << "LARGE LAMBDA";
        // }
            // if (i > 0) {
            //     LOG(INFO) << "idx " << i << "dk: " << (optimized_path->at(i).k - optimized_path->at(i-1).k) / (optimized_path->at(i).s - optimized_path->at(i-1).s);
            // }
    }
}

}