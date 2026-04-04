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
            std::vector<std::vector<std::shared_ptr<Joinery::Joint>>> jointsByGroup, 
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
        std::vector<std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::Joint>>> GetJointsByGroup() const
        {
            return this->_jointsByGroup;
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
            for (const auto& jointGroup : this->_jointsByGroup)
            {
                for (const auto& joint : jointGroup)
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
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ComputeOneIterationOfJointFaceTranslationsForOptimisation()
        {
            return this->_ComputeJointFaceTranslationsForOptimisation();
        }

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
         * @return A vector of pairs, where each pair consists of an anchor point (the center of a joint face),
         *  and a translation vector that indicates how much the joint face should be translated to better fit the skeleton and target area.
         */
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> _ComputeJointFaceTranslationsForOptimisation();

        std::vector<std::vector<std::shared_ptr<Joinery::Joint>>> _jointsByGroup;
        std::vector<Eigen::Vector3d> _skeleton;
        RoundwoodJoinery::PointCloud::PointCloud _pointCloud;
        double _referenceDiameter;
    };
}