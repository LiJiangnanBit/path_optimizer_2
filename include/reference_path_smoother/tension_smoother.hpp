//
// Created by ljn on 20-4-14.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_
#include <vector>
#include <cfloat>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "reference_path_smoother/reference_path_smoother.hpp"

namespace PathOptimizationNS {
class TensionSmoother : public ReferencePathSmoother {
 public:
    TensionSmoother() = delete;
    TensionSmoother(const std::vector<State> &input_points,
                    const State &start_state,
                    const Map &grid_map);
    ~TensionSmoother() override = default;

 private:
    bool smooth(std::shared_ptr<PathOptimizationNS::ReferencePath> reference_path) override;
    virtual bool osqpSmooth(const std::vector<double> &x_list,
                            const std::vector<double> &y_list,
                            const std::vector<double> &angle_list,
                            const std::vector<double> &k_list,
                            const std::vector<double> &s_list,
                            std::vector<double> *result_x_list,
                            std::vector<double> *result_y_list,
                            std::vector<double> *result_s_list);
    virtual void setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const;
    virtual void setConstraintMatrix(const std::vector<double> &x_list,
                                     const std::vector<double> &y_list,
                                     const std::vector<double> &angle_list,
                                     const std::vector<double> &k_list,
                                     const std::vector<double> &s_list,
                                     Eigen::SparseMatrix<double> *matrix_constraints,
                                     Eigen::VectorXd *lower_bound,
                                     Eigen::VectorXd *upper_bound) const;
};

}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_HPP_
