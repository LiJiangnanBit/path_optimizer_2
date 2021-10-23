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

std::vector<std::tuple<State, double, double>> ReferencePathImpl::display_abnormal_bounds() const {
    return display_set_;
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

void ReferencePathImpl::updateBoundsImproved(const PathOptimizationNS::Map &map) {
    if (reference_states_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    VehicleStateBound vehicle_state_bound;
    for (const auto &state : reference_states_) {
        // Front and rear bounds.
        State front_center(state.x + FLAGS_front_length * cos(state.heading),
                           state.y + FLAGS_front_length * sin(state.heading),
                           state.heading);
        State rear_center(state.x + FLAGS_rear_length * cos(state.heading),
                          state.y + FLAGS_rear_length * sin(state.heading),
                          state.heading);
        auto front_center_directional_projection = getDirectionalProjection(*x_s_,
                                                                            *y_s_,
                                                                            front_center.x,
                                                                            front_center.y,
                                                                            front_center.heading + M_PI_2,
                                                                            state.s + 5.0,
                                                                            state.s - 5.0);
        auto rear_center_directional_projection = getDirectionalProjection(*x_s_,
                                                                           *y_s_,
                                                                           rear_center.x,
                                                                           rear_center.y,
                                                                           rear_center.heading + M_PI_2,
                                                                           state.s + 5.0,
                                                                           state.s - 5.0);

        // Calculate boundaries.
        auto front_bound = getClearanceWithDirectionStrict(front_center_directional_projection, map);
        auto offset = global2Local(front_center, front_center_directional_projection).y;
        front_bound[0] += offset;
        front_bound[1] += offset;
        auto rear_bound = getClearanceWithDirectionStrict(rear_center_directional_projection, map);
        offset = global2Local(rear_center, rear_center_directional_projection).y;
        rear_bound[0] += offset;
        rear_bound[1] += offset;
        vehicle_state_bound.front.set(front_bound, front_center_directional_projection);
        vehicle_state_bound.rear.set(rear_bound, rear_center_directional_projection);
        if (isEqual(front_bound[0], front_bound[1]) || isEqual(rear_bound[0], rear_bound[1])) {
            LOG(INFO) << "Path is blocked at s: " << state.s;
            blocked_bound_.reset(new VehicleStateBound(vehicle_state_bound));
            break;
        }
        bounds_.emplace_back(vehicle_state_bound);
    }
    if (reference_states_.size() != bounds_.size()) {
        reference_states_.resize(bounds_.size());
    }
}

//void ReferencePathImpl::updateBounds(const Map &map) {
//    if (reference_states_.empty()) {
//        LOG(WARNING) << "Empty reference, updateBounds fail!";
//        return;
//    }
//    bounds_.clear();
//    for (const auto &state : reference_states_) {
//        // Circle centers.
//        State
//            c0(state.x + FLAGS_d1 * cos(state.z),
//               state.y + FLAGS_d1 * sin(state.z),
//               state.z),
//            c1(state.x + FLAGS_d2 * cos(state.z),
//               state.y + FLAGS_d2 * sin(state.z),
//               state.z),
//            c2(state.x + FLAGS_d3 * cos(state.z),
//               state.y + FLAGS_d3 * sin(state.z),
//               state.z),
//            c3(state.x + FLAGS_d4 * cos(state.z),
//               state.y + FLAGS_d4 * sin(state.z),
//               state.z);
//        // Calculate boundaries.
//        auto clearance_0 = getClearanceWithDirectionStrict(c0, map);
//        auto clearance_1 = getClearanceWithDirectionStrict(c1, map);
//        auto clearance_2 = getClearanceWithDirectionStrict(c2, map);
//        auto clearance_3 = getClearanceWithDirectionStrict(c3, map);
//        if (clearance_0[0] == clearance_0[1] ||
//            clearance_1[0] == clearance_1[1] ||
//            clearance_2[0] == clearance_2[1] ||
//            clearance_3[0] == clearance_3[1]) {
//            LOG(INFO) << "Path is blocked at s: " << state.s;
//            return;
//        }
//        VehicleStateBound covering_circle_bounds;
//        covering_circle_bounds.c0 = clearance_0;
//        covering_circle_bounds.c1 = clearance_1;
//        covering_circle_bounds.c2 = clearance_2;
//        covering_circle_bounds.c3 = clearance_3;
//        bounds_.emplace_back(covering_circle_bounds);
//    }
//    if (reference_states_.size() != bounds_.size()) {
//        reference_states_.resize(bounds_.size());
//    }
//    LOG(INFO) << "Boundary updated.";
//}

std::vector<double> ReferencePathImpl::getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
                                                                       const PathOptimizationNS::Map &map) {
    // TODO: too much repeated code!
    double left_bound = 0;
    double right_bound = 0;
    double delta_s = 0.3;
    double left_angle = constrainAngle(state.heading + M_PI_2);
    double right_angle = constrainAngle(state.heading - M_PI_2);
    static auto search_radius = 0.5;

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
        LOG(INFO) << "ref state has collision!";
        return {0.0, 0.0};
    }
//        DLOG(INFO) << "Using relative position to determine the direction to expand.";
//        // Use position to determine the direction.
//        auto closest_point{getProjection(*x_s_,
//                                         *y_s_,
//                                         state.x,
//                                         state.y,
//                                         max_s_)};
//        auto local_view{global2Local(state, closest_point)};
//        DLOG(INFO) << "closest point: " << closest_point.x << ", " << closest_point.y << "\n"
//                   << "state: " << state.x << ", " << state.y;
//        if (local_view.y < 0) {
//            DLOG(INFO) << "Choose right.";
//            // Expand to the right:
//            double right_s = 0;
//            for (int j = 0; j != n; ++j) {
//                right_s += delta_s;
//                double x = state.x + right_s * cos(right_angle);
//                double y = state.y + right_s * sin(right_angle);
//                grid_map::Position new_position(x, y);
//                double clearance = map.getObstacleDistance(new_position);
//                if (clearance > search_radius) {
//                    break;
//                }
//            }
//            left_bound = -right_s;
//            for (int j = 0; j != n; ++j) {
//                right_s += delta_s;
//                double x = state.x + right_s * cos(right_angle);
//                double y = state.y + right_s * sin(right_angle);
//                grid_map::Position new_position(x, y);
//                double clearance = map.getObstacleDistance(new_position);
//                if (clearance < search_radius) {
//                    break;
//                }
//            }
//            right_bound = -right_s + delta_s;
//        } else {
//            DLOG(INFO) << "Choose left.";
//            // Expand to the left:
//            double left_s = 0;
//            for (int j = 0; j != n; ++j) {
//                left_s += delta_s;
//                double x = state.x + left_s * cos(left_angle);
//                double y = state.y + left_s * sin(left_angle);
//                grid_map::Position new_position(x, y);
//                double clearance = map.getObstacleDistance(new_position);
//                if (clearance > search_radius) {
//                    break;
//                }
//            }
//            right_bound = left_s;
//            for (int j = 0; j != n; ++j) {
//                left_s += delta_s;
//                double x = state.x + left_s * cos(left_angle);
//                double y = state.y + left_s * sin(left_angle);
//                grid_map::Position new_position(x, y);
//                double clearance = map.getObstacleDistance(new_position);
//                if (clearance < search_radius) {
//                    break;
//                }
//            }
//            left_bound = left_s - delta_s;
//        }
//        DLOG(INFO) << left_bound << ", " << right_bound;
//    }
    // Search backward more precisely.
    double smaller_ds = 0.1;
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
    auto diff_radius = FLAGS_circle_radius - search_radius;
    left_bound -= diff_radius;
    right_bound += diff_radius;
    // Only one direction:
    if (left_bound * right_bound >= 0) {
        display_set_.emplace_back(std::make_tuple(state, left_bound, right_bound));
    }
    return {left_bound, right_bound};
}

bool ReferencePathImpl::buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger) {
    CHECK_LE(delta_s_smaller, delta_s_larger);
    if (isEqual(max_s_, 0.0)) {
        LOG(INFO) << "ref length is zero.";
        return false;
    }
    reference_states_.clear();
    const double large_k = 0.2;
    const double small_k = 0.08;
    double tmp_s = 0;
    while (tmp_s <= max_s_) {
        double x = (*x_s_)(tmp_s);
        double y = (*y_s_)(tmp_s);
        double h = getHeading(*x_s_, *y_s_, tmp_s);
        double k = getCurvature(*x_s_, *y_s_, tmp_s);
        reference_states_.emplace_back(x, y, h, k, tmp_s);
        // Use k to decide delta s.
        if (FLAGS_enable_dynamic_segmentation) {
            double k_share = fabs(k) > large_k ? 1 :
                             fabs(k) < small_k ? 0 : (fabs(k) - small_k) / (large_k - small_k);
            tmp_s += delta_s_larger - k_share * (delta_s_larger - delta_s_smaller);
        } else tmp_s += delta_s_larger;
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
}

std::shared_ptr<VehicleStateBound> ReferencePathImpl::isBlocked() const {
    return blocked_bound_;
}

}
