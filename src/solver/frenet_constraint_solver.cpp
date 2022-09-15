#include "solver/frenet_constraint_solver.hpp"
#include "data_struct/reference_path.hpp"
#include "config/planning_flags.hpp"
#include "data_struct/vehicle_state_frenet.hpp"
#include "tools/tools.hpp"
#include "tools/time_recorder.h"
namespace PathOptimizationNS {

FrenetConstraintSolver::FrenetConstraintSolver(const ReferencePath &reference_path,
               const VehicleState &vehicle_state,
               const std::vector<SlState> &input_path) : SimplifiedSolver(reference_path, vehicle_state, input_path) {}

double FrenetConstraintSolver::calculate_l_collision_coeff(const State& ref_state, const SlState& input_state, const VehicleStateBound::SingleBound& bound) const {
    double ds = bound.s - ref_state.s;

}
double FrenetConstraintSolver::calculate_dheading_collision_coeff(const State& ref_state, const SlState& input_state, const VehicleStateBound::SingleBound& bound) const {
    double ds = bound.s - ref_state.s;
    
}
double FrenetConstraintSolver::calculate_constrant_item(const State& ref_state, const SlState& input_state, const VehicleStateBound::SingleBound& bound) const {
    double ds = bound.s - ref_state.s;
    
}


void FrenetConstraintSolver::setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
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
    Eigen::Matrix<double, 2, 2> slack_coeff;
    slack_coeff << 1, 0, 0, 1;
    std::vector<std::pair<double, double>> constant_part_list;
    const double flat_kappa_threshold = 0.01;
    for (size_t i = 0; i < n_; ++i) {
        const auto& ref_state = ref_states[i];
        const auto& input_state = input_path_[i];
        const auto& bound = reference_path_.getBounds().at(i);
        if (i < precise_planning_size_) {
            Eigen::Matrix<double, 2, 2> collision_coeff;
            double fl = 0.0, fh = 0.0, rl = 0.0, rh = 0.0, fc = 0.0, rc = 0.0;
            // LOG(INFO) << "Front k " << bound.front.k << ", Rear k " << bound.rear.k;
            // Front constraints.
            if (fabs(bound.front.k) < 0.01) {
                fl = 1.0;
                fh = bound.front.s - ref_state.s;
            } else {
                const double ref_r = 1.0 / bound.front.k;
                const double x_c = bound.front.x + ref_r * cos(bound.front.heading + M_PI_2);
                const double y_c = bound.front.y + ref_r * sin(bound.front.heading + M_PI_2);
                const double dx = ref_state.x + input_state.l * cos(ref_state.heading + M_PI_2) + FLAGS_front_length * cos(input_state.heading) - x_c;
                const double dy = ref_state.y + input_state.l * sin(ref_state.heading + M_PI_2) + FLAGS_front_length * sin(input_state.heading) - y_c;
                const double dist = sqrt(pow(dx, 2) + pow(dy, 2));
                const double df_l = (dx * cos(ref_state.heading + M_PI_2) + dy * sin(ref_state.heading + M_PI_2)) / dist;
                const double df_h = (-dx * FLAGS_front_length * sin(input_state.heading) + dy * FLAGS_front_length * cos(input_state.heading)) / dist;
                const double f_const = dist - df_l * input_state.l - df_h * input_state.d_heading;
                if (bound.front.k > 0.0) {
                    fl = -df_l;
                    fh = -df_h;
                    fc = -f_const + ref_r;
                } else {
                    fl = df_l;
                    fh = df_h;
                    fc = f_const + ref_r;
                }
            }
            // Rear constraints.
            const double d_rear = FLAGS_rear_length * 0.5;
            if (fabs(bound.rear.k) < 0.01) {
                rl = 1.0;
                rh = bound.rear.s - ref_state.s;
            } else {
                const double ref_r = 1.0 / bound.rear.k;
                const double x_c = bound.rear.x + ref_r * cos(bound.rear.heading + M_PI_2);
                const double y_c = bound.rear.y + ref_r * sin(bound.rear.heading + M_PI_2);
                const double dx = ref_state.x + input_state.l * cos(ref_state.heading + M_PI_2) + d_rear * cos(input_state.heading) - x_c;
                const double dy = ref_state.y + input_state.l * sin(ref_state.heading + M_PI_2) + d_rear * sin(input_state.heading) - y_c;
                const double dist = sqrt(pow(dx, 2) + pow(dy, 2));
                const double df_l = (dx * cos(ref_state.heading + M_PI_2) + dy * sin(ref_state.heading + M_PI_2)) / dist;
                const double df_h = (-dx * d_rear * sin(input_state.heading) + dy * d_rear * cos(input_state.heading)) / dist;
                const double f_const = dist - df_l * input_state.l - df_h * input_state.d_heading;
                if (bound.rear.k > 0.0) {
                    rl = -df_l;
                    rh = -df_h;
                    rc = -f_const + ref_r;
                } else {
                    rl = df_l;
                    rh = df_h;
                    rc = f_const + ref_r;
                }
            }
            collision_coeff << fl, fh, rl, rh;
            // std::cout << "i: " << i << std::endl;
            // std::cout << collision_coeff << std::endl;
            // std::cout << "C: " << fc << ", " << rc << std::endl;
            constant_part_list.push_back(std::pair<double, double>(fc, rc)); 
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
            (*lower_bound)(precise_collision_idx + 2 * i, 0) = front_bounds.first - constant_part_list[i].first;
            (*upper_bound)(precise_collision_idx + 2 * i, 0) = front_bounds.second - constant_part_list[i].first;
            (*lower_bound)(precise_collision_idx + 2 * i + 1, 0) = rear_bounds.first - constant_part_list[i].second;
            (*upper_bound)(precise_collision_idx + 2 * i + 1, 0) = rear_bounds.second - constant_part_list[i].second;
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
}