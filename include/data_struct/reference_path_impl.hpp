//
// Created by ljn on 20-3-23.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_IMPL_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_IMPL_HPP_
#include <vector>
#include <tuple>
#include <memory>

namespace PathOptimizationNS {
class Map;
class Config;
class State;
class VehicleStateBound;
namespace tk {
class spline;
}

class ReferencePathImpl {
 public:
    ReferencePathImpl();
    ReferencePathImpl(const ReferencePathImpl &ref) = delete;
    ReferencePathImpl &operator=(const ReferencePathImpl &ref) = delete;

    const tk::spline &getXS() const;
    const tk::spline &getYS() const;
    // Set smoothed reference path.
    void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    // Set search result. It's used to calculate boundaries.
    void setOriginalSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    const tk::spline &getOriginalXS() const;
    const tk::spline &getOriginalYS() const;
    void clear();
    bool trimStates();
    std::size_t getSize() const;
    double getLength() const;
    void setLength(double s);
    const std::vector<State> &getReferenceStates() const;
    const std::vector<VehicleStateBound> &getBounds() const;
    void logBoundsInfo() const;
    // Calculate upper and lower bounds for each covering circle.
    void updateBoundsImproved(const Map &map);
    // Calculate reference_states_ from x_s_ and y_s_, given delta s.
    bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);
    bool buildReferenceFromStates(const std::vector<State> &states);
    std::shared_ptr<VehicleStateBound> isBlocked() const;

 private:
    std::vector<double> getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
                                                        const PathOptimizationNS::Map &map);
    State getApproxState(const State &original_state, const State &actual_state, double len) const;
    // Reference path spline representation.
    std::shared_ptr<tk::spline> x_s_;
    std::shared_ptr<tk::spline> y_s_;
    double max_s_{};
    // Raw ref line.
    std::shared_ptr<tk::spline> original_x_s_;
    std::shared_ptr<tk::spline> original_y_s_;
    double original_max_s_{};
    bool is_original_spline_set{false};
    // Divided smoothed path info.
    std::vector<State> reference_states_;
    std::vector<VehicleStateBound> bounds_;
    // Debug.
    std::shared_ptr<VehicleStateBound> blocked_bound_;
};
}

#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_IMPL_HPP_
