//
// Created by ljn on 20-3-10.
//

#include "solver/solver.hpp"
#include "data_struct/reference_path.hpp"
#include "data_struct/data_struct.hpp"

namespace PathOptimizationNS {

OsqpSolver::OsqpSolver(std::shared_ptr<ReferencePath> reference_path,
                       std::shared_ptr<VehicleState> vehicle_state) :
    horizon_(reference_path->getSize()),
    reference_path_(reference_path),
    vehicle_state_(vehicle_state),
    reference_interval_(0.0) {
    // Check some of the reference states to get the interval.
    const int check_num = 10;
    for (int i = 1; i < horizon_ && i < check_num; ++i) {
        reference_interval_ = 
            std::max(reference_interval_, reference_path_->getReferenceStates()[i].s - reference_path_->getReferenceStates()[i - 1].s);
    }
}

std::unique_ptr<OsqpSolver> OsqpSolver::create(std::string &type,
                                               std::shared_ptr<ReferencePath> reference_path,
                                               std::shared_ptr<VehicleState> vehicle_state) {
    // if (type == "K") {
    //     return std::unique_ptr<OsqpSolver>(new SolverKAsInput(reference_path, vehicle_state, horizon));
    // } else if (type == "KP") {
    //     return std::unique_ptr<OsqpSolver>(new SolverKpAsInput(reference_path, vehicle_state, horizon));
    // } else if (type == "KPC") {
    //     return std::unique_ptr<OsqpSolver>(new SolverKpAsInputConstrained(reference_path, vehicle_state, horizon));
    // } else {
    //     LOG(ERROR) << "No such solver!";
    //     return nullptr;
    // }
}

bool OsqpSolver::solve(std::vector<PathOptimizationNS::State> *optimized_path) {
    const auto &ref_states = reference_path_->getReferenceStates();
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(num_of_variables_);
    solver_.data()->setNumberOfConstraints(num_of_constraints_);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_of_variables_);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    // Set Hessian matrix.
    setHessianMatrix(&hessian);
    // Set state transition matrix, constraint matrix and bound vector.
    setConstraintMatrix(
        &linearMatrix,
        &lowerBound,
        &upperBound);
    // Input to solver.
    if (!solver_.data()->setHessianMatrix(hessian)) return false;
    if (!solver_.data()->setGradient(gradient)) return false;
    if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver_.data()->setLowerBound(lowerBound)) return false;
    if (!solver_.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver_.initSolver()) return false;
    if (!solver_.solve()) return false;
    const auto &QPSolution = solver_.getSolution();
    getOptimizedPath(QPSolution, optimized_path);
    return true;
}

}