#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include <Eigen/Dense>

#include "../Joint/Joint.hh"
#include "../Utils/Utils.hh"

namespace RoundwoodJoinery::Beam
{
    class Beam
    {
    public:
        Beam(double referenceDiameter, 
            std::vector<Joinery::JointGroup> jointGroups, 
            std::vector<Eigen::Vector3d> skeleton, 
            RoundwoodJoinery::PointCloud::PointCloud pointCloud);
            
        ~Beam() = default;

        /**
         * @brief Returns the reference diameter of the beam.
         * 
         * @return The reference diameter of the beam.
         */
        double GetReferenceDiameter() const
        {
            return this->_referenceDiameter;
        }

        /**
         * @brief Returns the joints associated with the beam.
         * 
         * @return A vector of shared pointers to the joints associated with the beam.
         */
        std::vector<Joinery::JointGroup> GetJointGroups() const
        {
            return this->_jointGroups;
        }

        /**
         * @brief Returns the skeleton of the beam.
         * 
         * @return The skeleton of the beam as a vector of 3D points.
         */
        std::vector<Eigen::Vector3d> GetSkeleton() const
        {
            return this->_skeleton;
        }

        /**
         * @brief Returns the point cloud associated with the beam.
         * 
         * @return The point cloud associated with the beam.
         */
        RoundwoodJoinery::PointCloud::PointCloud GetPointCloud() const
        {
            return this->_pointCloud;
        }

        /**
         * @brief Finds the closest point on the beam skeleton for each joint of the beam.
         * 
         */
        void FindJointClosestPointsOnSkeleton()
        {
            for (auto& jointGroup : this->_jointGroups)
            {
                for (auto& joint : jointGroup.GetJoints())
                {
                    Eigen::Vector3d jointCenter = joint->GetCenter();
                    Eigen::Vector3d correspondance = this->_FindClosestPointOnSkeleton(jointCenter);
                    joint->SetClosestPointOnSkeleton(correspondance);
                }
            }
        }

        /**
         * @brief Just a test function
         */
        std::vector<Eigen::Matrix4d> ComputeOneIterationOfJointFaceTranslationsForOptimisation()
        {
            std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> pointsAndTranslations = this->_ComputeJointFaceTranslationsForOptimisation();
            std::vector<Eigen::Matrix4d> transformations = RoundwoodJoinery::Utils::ComputeApproximatingTransformation(pointsAndTranslations);
            Eigen::Matrix4d meanTransformation = RoundwoodJoinery::Utils::ComputeMeanTransformation(transformations);
            std::vector<Eigen::Matrix4d> adaptedTransformations;

            for (size_t i = 0; i < transformations.size(); ++i)
            {
                Eigen::Matrix4d residualTransformation = meanTransformation.inverse() * transformations[i];
                Eigen::Vector3d jointGroupDOF = this->_jointGroups[i].GetDegreeOfFreedom().normalized();
                Eigen::Vector3d residualTranslation = residualTransformation.block<3,1>(0,3);
                Eigen::Matrix3d residualRotation = residualTransformation.block<3,3>(0,0);
                Eigen::Vector3d implicitTranslation = (residualRotation * this->_jointGroups[i].GetCentroid() + residualTranslation) - this->_jointGroups[i].GetCentroid();
                Eigen::Vector3d projectionOfImplicitTranslationOnDOF = implicitTranslation.dot(jointGroupDOF) * jointGroupDOF;
                Eigen::Matrix4d adaptedTransformation = meanTransformation;
                adaptedTransformation.block<3,1>(0,3) += projectionOfImplicitTranslationOnDOF;
                adaptedTransformations.push_back(adaptedTransformation);
            }
            return adaptedTransformations;
        }

        /**
         * @brief Iteratively computes and applies the transformations for each joint group to optimize their positions based on the skeleton and target areas of their joint faces.
         * 
         * @param maxIterations The maximum number of iterations to perform for the optimization process.
         * @param minRelativeTranslationRMSE The minimum relative translation root mean square error threshold to determine convergence of the optimization process. If the RMSE of the translations falls below this threshold, the optimization process will stop.
         * @return The vector of total transformations applied to each joint group. They have been applied and are returned for evaluation purposes.
         */
        std::vector<Eigen::Matrix4d> ComputeJointGroupOptimisation(int maxIterations, double minRelativeTranslationRMSE);

    private:


        /**
         * @brief Private method that for a given point finds the closest point on the beam skeleton.
         * 
         * @param point The point for which to find the closest point on the skeleton.
         * @return The closest point on the skeleton to the given point.
         */
        Eigen::Vector3d _FindClosestPointOnSkeleton(const Eigen::Vector3d& point);


        /**
         * @brief Private method that computes the translations of the joint faces for optimization purposes. 
         * This is based on the current positions of the joints, their closest points on the skeleton, and their joint faces' target areas.
         * 
         * @return A vector of vectors of pairs, where each inner vector corresponds to a group of joints,
         *  and each pair consists of an anchor point (a corner of a joint face) and a translation vector
         *  that indicates how much the joint face should be translated to better fit the skeleton and target area.
         */
        std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> _ComputeJointFaceTranslationsForOptimisation();

        std::vector<Joinery::JointGroup> _jointGroups;
        std::vector<Eigen::Vector3d> _skeleton;
        RoundwoodJoinery::PointCloud::PointCloud _pointCloud;
        double _referenceDiameter;
    };
}