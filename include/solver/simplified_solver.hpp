#ifndef PATH_OPTIMIZER_SIMPLIFIED_SOLVER_HPP
#define PATH_OPTIMIZER_SIMPLIFIED_SOLVER_HPP

#include "base_solver.hpp"

namespace PathOptimizationNS {

class SimplifiedSolver : public BaseSolver {

public:
    SimplifiedSolver(const ReferencePath &reference_path,
               const VehicleState &vehicle_state,
               const std::vector<SlState> &input_path);

    ~SimplifiedSolver() = default;

private:
    private:
    // Set Matrices for osqp solver.
    void setHessian(Eigen::SparseMatrix<double> *matrix_h) const override;

    void setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
                                Eigen::VectorXd *lower_bound,
                                Eigen::VectorXd *upper_bound) const override;

    void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                  std::vector<SlState> *optimized_path) const override;

};

}

#endif

