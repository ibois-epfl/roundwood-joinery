#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <string>
#include <optional>
#include <fstream>

#include <Eigen/Dense>

#include "../Joint/Joint.hh"
#include "../Utils/Utils.hh"

namespace RoundwoodJoinery::Beam
{
    class Beam
    {
    public:
        Beam(double referenceDiameter, 
            std::vector<std::shared_ptr<Joinery::JointGroup>> jointGroups, 
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
        std::vector<std::shared_ptr<Joinery::JointGroup>> GetJointGroups() const
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
                for (auto& joint : jointGroup->GetJoints())
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
        std::vector<Eigen::Matrix4d> ComputeOneIterationOfJointFaceTranslationsForOptimisation(double alphaForAreaComputations, double gain, double radiusSearch)
        {
            std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> pointsAndTranslations = this->_ComputeJointFaceTranslationsForOptimisation(alphaForAreaComputations, gain, radiusSearch);
            std::vector<Eigen::Matrix4d> transformations = RoundwoodJoinery::Utils::ComputeApproximatingTransformation(pointsAndTranslations);
            Eigen::Matrix4d meanTransformation = RoundwoodJoinery::Utils::ComputeCollectiveApproximatingTransformation(pointsAndTranslations);
            std::vector<Eigen::Matrix4d> adaptedTransformations;

            for (size_t i = 0; i < transformations.size(); ++i)
            {
                Eigen::Matrix4d residualTransformation = meanTransformation.inverse() * transformations[i];
                Eigen::Vector3d jointGroupDOF = this->_jointGroups[i]->GetDegreeOfFreedom().normalized();
                Eigen::Vector3d residualTranslation = residualTransformation.block<3,1>(0,3);
                Eigen::Matrix3d residualRotation = residualTransformation.block<3,3>(0,0);
                Eigen::Vector3d implicitTranslation = (residualRotation * this->_jointGroups[i]->GetCentroid() + residualTranslation) - this->_jointGroups[i]->GetCentroid();
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
         * @param alphaForAreaComputations The alpha parameter used in the computation of the current area and depths of the joint faces.
         * @param initialGain The initial gain factor that influences the magnitude of the translations applied to the joint faces. This gain may be adjusted during the optimization process.
         * @param radiusSearch The radius within which to search for points in the point cloud when computing the current area and depths of the joint faces.
         * @param outputFolderPath The path to the folder where the area ratios will be outputted for each iteration. If not provided, no output will be generated.
         * @return The vector of total transformations applied to each joint group. They have been applied and are returned for evaluation purposes.
         */
        std::vector<Eigen::Matrix4d> ComputeJointGroupOptimisation(int maxIterations, double minRelativeTranslationRMSE, double alphaForAreaComputations, double initialGain, double radiusSearch, std::optional<std::string> outputFolderPath);

        /**
         * @brief Computes the remaining sections in the beam for each joint group, and updates their internal state accordingly.
         */
        void ComputeRemainingSections();

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
         * @param alphaForAreaComputations A parameter used in the computation of the current area and depths of the joint faces.
         * @param gain A gain factor that influences the magnitude of the translations applied to the joint faces.
         * @param radiusSearch The radius within which to search for points in the point cloud when computing the current area and depths of the joint faces.
         * 
         * @return A vector of vectors of pairs, where each inner vector corresponds to a group of joints,
         *  and each pair consists of an anchor point (a corner of a joint face) and a translation vector
         *  that indicates how much the joint face should be translated to better fit the skeleton and target area.
         */
        std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> _ComputeJointFaceTranslationsForOptimisation(double alphaForAreaComputations, double gain, double radiusSearch);

        std::vector<std::shared_ptr<Joinery::JointGroup>> _jointGroups;
        std::vector<Eigen::Vector3d> _skeleton;
        RoundwoodJoinery::PointCloud::PointCloud _pointCloud;
        double _referenceDiameter;
    };
}