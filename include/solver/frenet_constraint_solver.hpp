#ifndef PATH_OPTIMIZER_FRENET_CONSTRAINT_SOLVER_HPP
#define PATH_OPTIMIZER_FRENET_CONSTRAINT_SOLVER_HPP

#include "simplified_solver.hpp"
#include "data_struct/data_struct.hpp"

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
    double calculate_l_collision_coeff(const State& ref_state, const SlState& input_state, const VehicleStateBound::SingleBound& bound) const;
    double calculate_dheading_collision_coeff(const State& ref_state, const SlState& input_state, const VehicleStateBound::SingleBound& bound) const;
    double calculate_constrant_item(const State& ref_state, const SlState& input_state, const VehicleStateBound::SingleBound& bound) const;
};

}

#endif

