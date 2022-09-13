#ifndef PATH_OPTIMIZER_FRENET_CONSTRAINT_SOLVER_HPP
#define PATH_OPTIMIZER_FRENET_CONSTRAINT_SOLVER_HPP

#include "simplified_solver.hpp"

namespace PathOptimizationNS {

class FrenetConstraintSolver : public SimplifiedSolver {

public:
    FrenetConstraintSolver(const ReferencePath &reference_path,
               const VehicleState &vehicle_state,
               const std::vector<SlState> &input_path);

    ~FrenetConstraintSolver() = default;

private:
    void setConstraints(Eigen::SparseMatrix<double> *matrix_constraints,
                                Eigen::VectorXd *lower_bound,
                                Eigen::VectorXd *upper_bound) const override;

};

}

#endif

