//
// Created by ljn on 20-3-23.
//
#include <cfloat>
#include <glog/logging.h>
#include "data_struct/reference_path_impl.hpp"
#include <tools/Map.hpp>
#include "tools/tools.hpp"
#include "tools/spline.h"
#include "data_struct/data_struct.hpp"
#include "config/planning_flags.hpp"

namespace PathOptimizationNS {

ReferencePathImpl::ReferencePathImpl() :
    x_s_(new tk::spline),
    y_s_(new tk::spline),
    original_x_s_(new tk::spline),
    original_y_s_(new tk::spline),
    blocked_bound_(nullptr) {}

const tk::spline &ReferencePathImpl::getXS() const {
    return *x_s_;
}

const tk::spline &ReferencePathImpl::getYS() const {
    return *y_s_;
}

void ReferencePathImpl::setSpline(const tk::spline &x_s,
                                  const tk::spline &y_s,
                                  double max_s) {
    *x_s_ = x_s;
    *y_s_ = y_s;
    max_s_ = max_s;
}

void ReferencePathImpl::setOriginalSpline(const PathOptimizationNS::tk::spline &x_s,
                                          const PathOptimizationNS::tk::spline &y_s,
                                          double max_s) {
    *original_x_s_ = x_s;
    *original_y_s_ = y_s;
    original_max_s_ = max_s;
    is_original_spline_set = true;
}

const tk::spline &ReferencePathImpl::getOriginalXS() const {
    return *original_x_s_;
}

const tk::spline &ReferencePathImpl::getOriginalYS() const {
    return *original_y_s_;
}

void ReferencePathImpl::clear() {
    max_s_ = 0;
    reference_states_.clear();
    bounds_.clear();
}

std::size_t ReferencePathImpl::getSize() const {
    return reference_states_.size();
}

bool ReferencePathImpl::trimStates() {
    if (bounds_.empty() || reference_states_.empty() || bounds_.size() >= reference_states_.size())
        return false;
    reference_states_.resize(bounds_.size());
    return true;
}

double ReferencePathImpl::getLength() const {
    return max_s_;
}

void ReferencePathImpl::setLength(double s) {
    max_s_ = s;
}

const std::vector<State> &ReferencePathImpl::getReferenceStates() const {
    return reference_states_;
}

const std::vector<VehicleStateBound> &ReferencePathImpl::getBounds() const {
    return bounds_;
}

void ReferencePathImpl::logBoundsInfo() const {
    for (const auto &bound : bounds_) {
        const auto &front = bound.front;
        const auto &rear = bound.rear;
        LOG(INFO) << "front: (" << front.x << ", " << front.y << "), bound: (" << front.lb << ", " << front.ub << ").";
        LOG(INFO) << "rear: (" << rear.x << ", " << rear.y << "), bound: (" << rear.lb << ", " << rear.ub << ").";
    }
}

State ReferencePathImpl::getApproxState(const State &original_state, const State &actual_state, double len) const {
    // Point on reference.
    double x = (*x_s_)(original_state.s + len);
    double y = (*y_s_)(original_state.s + len);
    State v1, v2;
    v1.x = actual_state.x - original_state.x;
    v1.y = actual_state.y - original_state.y;
    v2.x = x - original_state.x;
    v2.y = y - original_state.y;
    double proj = (v1.x * v2.x + v1.y * v2.y) / std::max(0.001, sqrt(pow(v1.x, 2) + pow(v1.y, 2)));
    double move_dis = fabs(len) - proj;
    // Move.
    State ret;
    int sign = len >= 0.0 ? 1 : -1;
    ret.x = x + sign * move_dis * cos(original_state.heading);
    ret.y = y + sign * move_dis * sin(original_state.heading);
    ret.heading = original_state.heading;
    return ret;
}

void ReferencePathImpl::updateBoundsOnInputStates(const Map &map, std::vector<SlState> &input_sl_states) {
    CHECK_LE(input_sl_states.size(), reference_states_.size());
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    VehicleStateBound vehicle_state_bound;
    const double front_circle_radius = 0.5 * FLAGS_car_width;
    const double rear_circle_radius = 0.5 * sqrt(FLAGS_rear_length * FLAGS_rear_length + FLAGS_car_width * FLAGS_car_width);
    for (size_t i = 0; i < input_sl_states.size(); ++i) {
        const State& ref_state = reference_states_.at(i);
        const SlState& input_state = input_sl_states.at(i);
        const double front_length_new = FLAGS_front_length * cos(input_state.d_heading);
        const double rear_length_new = FLAGS_rear_length * 0.5 * cos(input_state.d_heading);
        // Front and rear bounds.
        State front_center(ref_state.x + front_length_new * cos(ref_state.heading),
                           ref_state.y + front_length_new * sin(ref_state.heading),
                           ref_state.heading);
        State rear_center(ref_state.x + rear_length_new * cos(ref_state.heading),
                          ref_state.y + rear_length_new * sin(ref_state.heading),
                          ref_state.heading);
        auto front_center_directional_projection = getDirectionalProjectionByNewton(*x_s_,
                                                                                    *y_s_,
                                                                                    front_center.x,
                                                                                    front_center.y,
                                                                                    ref_state.heading + M_PI_2,
                                                                                    ref_state.s + 5.0,
                                                                                    ref_state.s + FLAGS_front_length);
        auto rear_center_directional_projection = getDirectionalProjectionByNewton(*x_s_,
                                                                                   *y_s_,
                                                                                   rear_center.x,
                                                                                   rear_center.y,
                                                                                   ref_state.heading + M_PI_2,
                                                                                   ref_state.s + 5.0,
                                                                                   ref_state.s + FLAGS_rear_length);
        front_center_directional_projection.heading = rear_center_directional_projection.heading = ref_state.heading;
        // Calculate boundaries.
        auto front_bound = getClearanceWithDirectionStrict(front_center_directional_projection, map, front_circle_radius);
        auto offset = global2Local(front_center, front_center_directional_projection).y;
        front_bound[0] += offset;
        front_bound[1] += offset;
        auto rear_bound = getClearanceWithDirectionStrict(rear_center_directional_projection, map, rear_circle_radius);
        offset = global2Local(rear_center, rear_center_directional_projection).y;
        rear_bound[0] += offset;
        rear_bound[1] += offset;
        auto center_bound = getClearanceWithDirectionStrict(ref_state, map, front_circle_radius);
        vehicle_state_bound.front.set(front_bound, front_center);
        vehicle_state_bound.rear.set(rear_bound, rear_center);
        vehicle_state_bound.center.set(center_bound, ref_state);
        bounds_.emplace_back(vehicle_state_bound);
    }
}

void ReferencePathImpl::updateBoundsImproved(const PathOptimizationNS::Map &map) {
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    VehicleStateBound vehicle_state_bound;
    const double front_circle_radius = 0.5 * FLAGS_car_width;
    const double rear_circle_radius = 0.5 * sqrt(FLAGS_rear_length * FLAGS_rear_length + FLAGS_car_width * FLAGS_car_width);
    for (const auto &state : reference_states_) {
        // Front and rear bounds.
        State front_center(state.x + FLAGS_front_length * cos(state.heading),
                           state.y + FLAGS_front_length * sin(state.heading),
                           state.heading);
        State rear_center(state.x + FLAGS_rear_length * 0.5 * cos(state.heading),
                          state.y + FLAGS_rear_length * 0.5 * sin(state.heading),
                          state.heading);
        auto front_center_directional_projection = getDirectionalProjectionByNewton(*x_s_,
                                                                                    *y_s_,
                                                                                    front_center.x,
                                                                                    front_center.y,
                                                                                    front_center.heading + M_PI_2,
                                                                                    state.s + 5.0,
                                                                                    state.s + FLAGS_front_length);
        auto rear_center_directional_projection = getDirectionalProjectionByNewton(*x_s_,
                                                                                   *y_s_,
                                                                                   rear_center.x,
                                                                                   rear_center.y,
                                                                                   rear_center.heading + M_PI_2,
                                                                                   state.s + 5.0,
                                                                                   state.s + FLAGS_rear_length);
        front_center_directional_projection.heading = rear_center_directional_projection.heading = state.heading;
        // Calculate boundaries.
        auto front_bound = getClearanceWithDirectionStrict(front_center_directional_projection, map, front_circle_radius);
        auto offset = global2Local(front_center, front_center_directional_projection).y;
        front_bound[0] += offset;
        front_bound[1] += offset;
        auto rear_bound = getClearanceWithDirectionStrict(rear_center_directional_projection, map, rear_circle_radius);
        offset = global2Local(rear_center, rear_center_directional_projection).y;
        rear_bound[0] += offset;
        rear_bound[1] += offset;
        auto center_bound = getClearanceWithDirectionStrict(state, map, front_circle_radius);
        vehicle_state_bound.front.set(front_bound, front_center);
        vehicle_state_bound.rear.set(rear_bound, rear_center);
        vehicle_state_bound.center.set(center_bound, state);
        bounds_.emplace_back(vehicle_state_bound);
    }
}

void ReferencePathImpl::updateBoundsOnInputStatesFrenet(const Map &map, std::vector<SlState> &input_sl_states) {
    CHECK_LE(input_sl_states.size(), reference_states_.size());
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    VehicleStateBound vehicle_state_bound;
    const double front_circle_radius = 0.5 * FLAGS_car_width;
    const double rear_circle_radius = 0.5 * sqrt(FLAGS_rear_length * FLAGS_rear_length + FLAGS_car_width * FLAGS_car_width);
    for (size_t i = 0; i < input_sl_states.size(); ++i) {
        const State& ref_state = reference_states_.at(i);
        const SlState& input_state = input_sl_states.at(i);
        // Front and rear bounds.
        State front_center(input_state.x + FLAGS_front_length * cos(input_state.heading),
                           input_state.y + FLAGS_front_length * sin(input_state.heading),
                           input_state.heading);
        State rear_center(input_state.x + FLAGS_rear_length * 0.5 * cos(input_state.heading),
                          input_state.y + FLAGS_rear_length * 0.5 * sin(input_state.heading),
                          input_state.heading);
        auto front_center_projection = getProjectionByNewton(*x_s_,
                                                            *y_s_,
                                                            front_center.x,
                                                            front_center.y,
                                                            ref_state.s + 5.0,
                                                            ref_state.s + FLAGS_front_length);
        auto rear_center_projection = getProjectionByNewton(*x_s_,
                                                            *y_s_,
                                                            rear_center.x,
                                                            rear_center.y,
                                                            ref_state.s + 5.0,
                                                            ref_state.s + FLAGS_rear_length);
        // Calculate boundaries.
        auto front_bound = getClearanceWithDirectionStrict(front_center_projection, map, front_circle_radius);
        auto rear_bound = getClearanceWithDirectionStrict(rear_center_projection, map, rear_circle_radius);
        auto center_bound = getClearanceWithDirectionStrict(ref_state, map, front_circle_radius);
        vehicle_state_bound.front.set(front_bound, front_center_projection);
        vehicle_state_bound.rear.set(rear_bound, rear_center_projection);
        vehicle_state_bound.center.set(center_bound, ref_state);
        bounds_.emplace_back(vehicle_state_bound);
    }
}

void ReferencePathImpl::updateBoundsImprovedFrenet(const PathOptimizationNS::Map &map) {
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    VehicleStateBound vehicle_state_bound;
    const double front_circle_radius = 0.5 * FLAGS_car_width;
    const double rear_circle_radius = 0.5 * sqrt(FLAGS_rear_length * FLAGS_rear_length + FLAGS_car_width * FLAGS_car_width);
    for (const auto &state : reference_states_) {
        // Front and rear bounds.
        State front_center(state.x + FLAGS_front_length * cos(state.heading),
                           state.y + FLAGS_front_length * sin(state.heading),
                           state.heading);
        State rear_center(state.x + FLAGS_rear_length * 0.5 * cos(state.heading),
                          state.y + FLAGS_rear_length * 0.5 * sin(state.heading),
                          state.heading);
        auto front_center_directional_projection = getProjectionByNewton(*x_s_,
                                                                        *y_s_,
                                                                        front_center.x,
                                                                        front_center.y,
                                                                        state.s + 5.0,
                                                                        state.s + FLAGS_front_length);
        auto rear_center_directional_projection = getProjectionByNewton(*x_s_,
                                                                        *y_s_,
                                                                        rear_center.x,
                                                                        rear_center.y,
                                                                        state.s + 5.0,
                                                                        state.s + FLAGS_rear_length);
        // Calculate boundaries.
        auto front_bound = getClearanceWithDirectionStrict(front_center_directional_projection, map, front_circle_radius);
        auto rear_bound = getClearanceWithDirectionStrict(rear_center_directional_projection, map, rear_circle_radius);
        auto center_bound = getClearanceWithDirectionStrict(state, map, front_circle_radius);
        vehicle_state_bound.front.set(front_bound, front_center_directional_projection);
        vehicle_state_bound.rear.set(rear_bound, rear_center_directional_projection);
        vehicle_state_bound.center.set(center_bound, state);
        bounds_.emplace_back(vehicle_state_bound);
    }
}

std::vector<double> ReferencePathImpl::getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
                                                                       const PathOptimizationNS::Map &map,
                                                                       double radius) {
    // TODO: too much repeated code!
    double left_bound = 0;
    double right_bound = 0;
    double delta_s = 0.3;
    double left_angle = constrainAngle(state.heading + M_PI_2);
    double right_angle = constrainAngle(state.heading - M_PI_2);
    static auto search_radius = 0.6;

    auto n = static_cast<size_t >(6.0 / delta_s);
    // Check if the original position is collision free.
    grid_map::Position original_position(state.x, state.y);
    auto original_clearance = map.getObstacleDistance(original_position);
    if (original_clearance > search_radius) {
        // Normal case:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance < search_radius) {
                break;
            }
        }
        double left_s = 0;
        for (size_t j = 0; j != n; ++j) {
            left_s += delta_s;
            double x = state.x + left_s * cos(left_angle);
            double y = state.y + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.getObstacleDistance(new_position);
            if (clearance < search_radius) {
                break;
            }
        }
        right_bound = -(right_s - delta_s);
        left_bound = left_s - delta_s;
    } else {
        LOG(INFO) << "ref state is close to obstacles! xy: " << state.x << ", " << state.y;
        return {0.0, 0.0};
    }
    // Search backward more precisely.
    double smaller_ds = 0.05;
    for (int i = 1; i != static_cast<int>(delta_s / smaller_ds); ++i) {
        left_bound += smaller_ds;
        grid_map::Position position(
            state.x + left_bound * cos(left_angle),
            state.y + left_bound * sin(left_angle)
        );
        if (map.getObstacleDistance(position) < search_radius) {
            left_bound -= smaller_ds;
            break;
        }
    }
    for (int i = 1; i != static_cast<int>(delta_s / smaller_ds); ++i) {
        right_bound -= smaller_ds;
        grid_map::Position position(
            state.x + right_bound * cos(right_angle),
            state.y + right_bound * sin(right_angle)
        );
        if (map.getObstacleDistance(position) < search_radius) {
            right_bound += smaller_ds;
            break;
        }
    }
    auto diff_radius = radius - search_radius;
    left_bound -= diff_radius;
    right_bound += diff_radius;
    return {left_bound, right_bound};
}

bool ReferencePathImpl::buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger) {
    // CHECK_LE(delta_s_smaller, delta_s_larger);
    // if (isEqual(max_s_, 0.0)) {
    //     LOG(INFO) << "ref length is zero.";
    //     return false;
    // }
    // reference_states_.clear();
    // const double large_k = 0.2;
    // const double small_k = 0.08;
    // double tmp_s = 0;
    // while (tmp_s <= max_s_) {
    //     double step = tmp_s > FLAGS_rough_sampling_begin_s ? delta_s_larger : delta_s_smaller;
    //     double x = (*x_s_)(tmp_s);
    //     double y = (*y_s_)(tmp_s);
    //     double h = getHeading(*x_s_, *y_s_, tmp_s);
    //     double k = getCurvature(*x_s_, *y_s_, tmp_s);
    //     reference_states_.emplace_back(x, y, h, k, tmp_s);
    //     // Use k to decide delta s.
    //     if (FLAGS_enable_dynamic_segmentation) {
    //         double k_share = fabs(k) > large_k ? 1 :
    //                          fabs(k) < small_k ? 0 : (fabs(k) - small_k) / (large_k - small_k);
    //         tmp_s += step - k_share * (0.5 * step);
    //     } else tmp_s += step;
    // }
    // return true;
    if (isEqual(max_s_, 0.0)) {
        LOG(INFO) << "ref length is zero.";
        return false;
    }
    reference_states_.clear();
    const int max_pts = 170;
    const double step = std::max(FLAGS_output_spacing, max_s_ / max_pts);
    double tmp_s = 0;
    while (tmp_s <= max_s_) {
        double x = (*x_s_)(tmp_s);
        double y = (*y_s_)(tmp_s);
        double h = getHeading(*x_s_, *y_s_, tmp_s);
        double k = getCurvature(*x_s_, *y_s_, tmp_s);
        reference_states_.emplace_back(x, y, h, k, tmp_s);
        tmp_s += step;
    }
    return true;
}

bool ReferencePathImpl::buildReferenceFromStates(const std::vector<PathOptimizationNS::State> &states) {
    reference_states_ = states;
    std::vector<double> x_set, y_set, s_set;
    for (const auto &state: states) {
        x_set.emplace_back(state.x);
        y_set.emplace_back(state.y);
        s_set.emplace_back(state.s);
    }
    tk::spline x_s, y_s;
    x_s.set_points(s_set, x_set);
    y_s.set_points(s_set, y_set);
    setSpline(x_s, y_s, max_s_);
    return true;
}

std::shared_ptr<VehicleStateBound> ReferencePathImpl::isBlocked() const {
    return blocked_bound_;
}

}
