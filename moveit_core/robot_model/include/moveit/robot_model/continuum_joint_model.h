#pragma once

#include <moveit/robot_model/joint_model.h>

namespace moveit {
    namespace core {
/** \brief A continuum joint */
        class ContinuumJointModel : public JointModel {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            ContinuumJointModel(const std::string &name);

            void getVariableDefaultPositions(double *values, const Bounds &other_bounds) const override;

            void getVariableRandomPositions(random_numbers::RandomNumberGenerator &rng, double *values,
                                            const Bounds &other_bounds) const override;

            void getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator &rng, double *values,
                                                  const Bounds &other_bounds, const double *near,
                                                  const double distance) const override;

            bool enforcePositionBounds(double *values, const Bounds &other_bounds) const override;

            bool
            satisfiesPositionBounds(const double *values, const Bounds &other_bounds, double margin) const override;

            void interpolate(const double *from, const double *to, const double t, double *state) const override;

            unsigned int getStateSpaceDimension() const override;

            double getMaximumExtent(const Bounds &other_bounds) const override;

            double distance(const double *values1, const double *values2) const override;

            void computeTransform(const double *joint_values, Eigen::Isometry3d &transf) const override;

            void computeVariablePositions(const Eigen::Isometry3d &transf, double *joint_values) const override;


        };
    }  // namespace core
}  // namespace moveit
