//
// Created by ljn on 20-3-23.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_HPP_
#include <memory>
#include <vector>
#include <tuple>

namespace PathOptimizationNS {
class Map;
class Config;
class State;
class VehicleStateBound;
namespace tk {
class spline;
}
class ReferencePathImpl;

class ReferencePath {
 public:
    ReferencePath();
    const tk::spline &getXS() const;
    const tk::spline &getYS() const;
    double getXS(double s) const;
    double getYS(double s) const;
    void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    void setOriginalSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    void clear();
    std::size_t getSize() const;
    double getLength() const;
    void setLength(double s);
    const std::vector<State> &getReferenceStates() const;
    const std::vector<VehicleStateBound> &getBounds() const;
    void logBoundsInfo() const;
    // Calculate upper and lower bounds for each covering circle.
    void updateBounds(const Map &map);
    // Calculate reference_states_ from x_s_ and y_s_, given delta s.
    bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);
    bool buildReferenceFromStates(const std::vector<PathOptimizationNS::State> &states);
    std::shared_ptr<VehicleStateBound> isBlocked() const;
 private:
    std::shared_ptr<ReferencePathImpl> reference_path_impl_;
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_HPP_
