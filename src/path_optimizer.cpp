//
// Created by ljn on 19-8-16.
//
#include <iostream>
#include <cmath>
#include <ctime>
#include "path_optimizer_2/path_optimizer.hpp"
#include "config/planning_flags.hpp"
#include "glog/logging.h"
#include "tools/time_recorder.h"
#include "reference_path_smoother/reference_path_smoother.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "data_struct/data_struct.hpp"
#include "data_struct/vehicle_state_frenet.hpp"
#include "tools/collosion_checker.hpp"
#include "tools/Map.hpp"
#include "tools/spline.h"
#include "solver/base_solver.hpp"
#include "solver/simplified_solver.hpp"
#include "solver/frenet_constraint_solver.hpp"
#include "tinyspline_ros/tinysplinecpp.h"
#include "reference_path_smoother/tension_smoother.hpp"

namespace PathOptimizationNS {

PathOptimizer::PathOptimizer(const State &start_state,
                             const State &end_state,
                             const grid_map::GridMap &map) :
    grid_map_(std::make_shared<Map>(map)),
    collision_checker_(std::make_shared<CollisionChecker>(map)),
    reference_path_(std::make_shared<ReferencePath>()),
    vehicle_state_(std::make_shared<VehicleState>(start_state, end_state, 0.0, 0.0)) {
}

bool PathOptimizer::solve(const std::vector<State> &reference_points, std::vector<SlState> *final_path) {
    CHECK_NOTNULL(final_path);
    if (reference_points.empty()) {
        LOG(ERROR) << "Empty input, quit path optimization";
        return false;
    }

    TimeRecorder time_recorder("Outer Solve Function");
    time_recorder.recordTime("reference path smoothing");
    // Smooth reference path.
    // TODO: refactor this part!
    reference_path_->clear();
    auto reference_path_smoother = ReferencePathSmoother::create(FLAGS_smoothing_method,
                                                                 reference_points,
                                                                 vehicle_state_->getStartState(),
                                                                 *grid_map_);
    if (!reference_path_smoother->solve(reference_path_)) {
        LOG(ERROR) << "Path optimization FAILED!";
        return false;
    }

    time_recorder.recordTime("reference path segmentation");
    // Divide reference path into segments;
    if (!processReferencePath()) {
        LOG(ERROR) << "Path optimization FAILED!";
        return false;
    }

    time_recorder.recordTime("path optimization");
    // Optimize.
    if (!optimizePath(final_path)) {
        LOG(ERROR) << "Path optimization FAILED!";
        return false;
    }
    time_recorder.recordTime("end");
    time_recorder.printTime();
    return true;
}

void PathOptimizer::processInitState() {
    // Calculate the initial deviation and the angle difference.
    State init_point(reference_path_->getXS(0.0),
                     reference_path_->getYS(0.0),
                     getHeading(reference_path_->getXS(), reference_path_->getYS(), 0.0));
    auto first_point_local = global2Local(vehicle_state_->getStartState(), init_point);
    // In reference smoothing, the closest point to the vehicle is found and set as the
    // first point. So the distance here is simply the initial offset.
    double min_distance = distance(vehicle_state_->getStartState(), init_point);
    double initial_offset = first_point_local.y < 0.0 ? min_distance : -min_distance;
    double initial_heading_error = constrainAngle(vehicle_state_->getStartState().heading - init_point.heading);
    vehicle_state_->setInitError(initial_offset, initial_heading_error);
}

void PathOptimizer::setReferencePathLength() {
    State end_ref_state(reference_path_->getXS(reference_path_->getLength()),
                        reference_path_->getYS(reference_path_->getLength()),
                        getHeading(reference_path_->getXS(), reference_path_->getYS(), reference_path_->getLength()));
    auto vehicle_target_to_end_ref_state = global2Local(end_ref_state, vehicle_state_->getTargetState());
    // Target state is out of ref line.
    if (vehicle_target_to_end_ref_state.x > 0.0) {
        return;
    }
    auto target_projection = getProjection(reference_path_->getXS(),
                                           reference_path_->getYS(),
                                           vehicle_state_->getTargetState().x,
                                           vehicle_state_->getTargetState().y,
                                           reference_path_->getLength(),
                                           0.0);
    reference_path_->setLength(target_projection.s);
}

bool PathOptimizer::processReferencePath() {
    if (isEqual(reference_path_->getLength(), 0.0)) {
        LOG(ERROR) << "Smoothed path is empty!";
        return false;
    }

    processInitState();
    // If the start heading differs a lot with the ref path, quit.
    if (fabs(vehicle_state_->getInitError().back()) > 75 * M_PI / 180) {
        LOG(ERROR) << "Initial psi error is larger than 75°, quit path optimization!";
        return false;
    }
    setReferencePathLength();

    reference_path_->buildReferenceFromSpline(FLAGS_output_spacing, FLAGS_output_spacing * 2);
    reference_path_->updateBoundsFrenet(*grid_map_);
    return true;
}

bool PathOptimizer::optimizePath(std::vector<SlState> *final_path) {
    CHECK_NOTNULL(final_path);
    final_path->clear();
    intermediate_path_.clear();
    // TODO: remove arg: iter_num.
    std::vector<SlState> input_path;
    for (const auto &ref_state : reference_path_->getReferenceStates()) {
        SlState input_state;
        input_state.x = ref_state.x;
        input_state.y = ref_state.y;
        input_state.heading = ref_state.heading;
        input_state.s = ref_state.s;
        input_state.k = ref_state.k;
        input_path.push_back(input_state);
    }
    FrenetConstraintSolver solver(*reference_path_, *vehicle_state_, input_path);
    TimeRecorder time_recorder("Optimize Path Function");
    // Solve with soft collision constraints.
    time_recorder.recordTime("Pre solving");
    if (!solver.solve(&intermediate_path_)) {
        LOG(ERROR) << "Pre solving failed!";
        reference_path_->logBoundsInfo();
        return false;
    }
    *final_path = intermediate_path_;
    time_recorder.recordTime("Update ref");
    reference_path_->updateBoundsOnInputStatesFrenet(*grid_map_, intermediate_path_);
    // Solve.
    time_recorder.recordTime("Solving");
    // BaseSolver post_solver(*reference_path_, *vehicle_state_, *final_path);
    // if (!post_solver.solve(final_path)) {
    if (!solver.updateProblemFormulationAndSolve(intermediate_path_, final_path)) {
        LOG(ERROR) << "Solving failed!";
        reference_path_->logBoundsInfo();
        return false;
    }
    time_recorder.recordTime("end");
    time_recorder.printTime();
    return true;
}

const ReferencePath &PathOptimizer::getReferencePath() const {
    return *reference_path_;
}

const std::vector<SlState> &PathOptimizer::getIntermediatePath() const {
    return intermediate_path_;
}
}
