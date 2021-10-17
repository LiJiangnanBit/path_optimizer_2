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

class BaseSolver {

 public:
    BaseSolver() = delete;

    BaseSolver(std::shared_ptr<ReferencePath> reference_path,
               std::shared_ptr<VehicleState> vehicle_state);

    virtual ~BaseSolver() = default;

    static std::unique_ptr<BaseSolver> create(std::string &type,
                                              std::shared_ptr<ReferencePath> reference_path,
                                              std::shared_ptr<VehicleState> vehicle_state);

    virtual bool solve(std::vector<State> *optimized_path);

 private:
    // Set Matrices for osqp solver.
    virtual void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const;

    virtual void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                     Eigen::VectorXd *lower_bound,
                                     Eigen::VectorXd *upper_bound) const;
    virtual void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<State> *optimized_path) const;

 protected:
    const size_t horizon_{};
    std::shared_ptr<ReferencePath> reference_path_;
    std::shared_ptr<VehicleState> vehicle_state_;
    OsqpEigen::Solver solver_;
    double reference_interval_;
    int num_of_variables_, num_of_constraints_;

};

}
#endif //PATH_OPTIMIZER_SOLVER_HPP
