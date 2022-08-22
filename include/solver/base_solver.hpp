//
// Created by ljn on 20-3-10.
//

#ifndef PATH_OPTIMIZER_SOLVER_HPP
#define PATH_OPTIMIZER_SOLVER_HPP

#include <Eigen/Dense>
#include <memory>
#include <OsqpEigen/OsqpEigen.h>
#include "glog/logging.h"

namespace PathOptimizationNS {

class Config;
class ReferencePath;
class VehicleState;
class State;
class SlState;

class BaseSolver {

 public:
    BaseSolver() = delete;

    BaseSolver(std::shared_ptr<const ReferencePath> reference_path,
               std::shared_ptr<const VehicleState> vehicle_state,
               std::shared_ptr<const std::vector<SlState>> input_path);

    virtual ~BaseSolver() = default;

    static std::unique_ptr<BaseSolver> create(std::string &type,
                                              std::shared_ptr<ReferencePath> reference_path,
                                              std::shared_ptr<VehicleState> vehicle_state);

    virtual bool solve(std::vector<State> *optimized_path);

 private:
    // Set Matrices for osqp solver.
    virtual void setCost(Eigen::SparseMatrix<double> *matrix_h) const;

    virtual void setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
                                Eigen::VectorXd *lower_bound,
                                Eigen::VectorXd *upper_bound) const;
    virtual void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<State> *optimized_path) const;
    std::pair<double, double> getSoftBounds(double lb, double ub, double safety_margin) const;

 protected:
    // Num of knots.
    const size_t n_{};
    size_t state_size_{};
    size_t control_size_{};
    size_t slack_size_{};
    size_t vars_size_{};
    size_t cons_size_{};
    size_t precise_planning_size_{};
    std::shared_ptr<const ReferencePath> reference_path_;
    std::shared_ptr<const VehicleState> vehicle_state_;
    std::shared_ptr<const std::vector<SlState>> input_path_;
    OsqpEigen::Solver solver_;
    double reference_interval_;
};

}
#endif //PATH_OPTIMIZER_SOLVER_HPP
