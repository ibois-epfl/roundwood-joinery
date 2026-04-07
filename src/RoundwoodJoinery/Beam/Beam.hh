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
         * @brief Sets the relative degrees of freedom between two joint groups. 
         * It is a symmetric relationship, so it updates both [index_1][index_2] and [index_2][index_1] in the matrix.
         * 
         * @param index_1 The index of the first joint group.
         * @param index_2 The index of the second joint group.
         * @param degreesOfFreedom The translation degree of freedom to set between the two joint groups, represented as a 3D vector.
         */
        void SetDegreesOfFreedomBetweenJointGroups(size_t index_1, size_t index_2, Eigen::Vector3d degreesOfFreedom)
        {
            if (index_1 >= this->_jointGroups.size() || index_2 >= this->_jointGroups.size())
            {
                std::cerr << "Error: Joint group index out of range when setting degrees of freedom." << std::endl;
                return;
            }
            this->_relativeDegreesOfFreedom[index_1][index_2] = degreesOfFreedom;
            this->_relativeDegreesOfFreedom[index_2][index_1] = degreesOfFreedom;
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
        std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> ComputeOneIterationOfJointFaceTranslationsForOptimisation()
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
         * @return A vector of vectors of pairs, where each inner vector corresponds to a group of joints,
         *  and each pair consists of an anchor point (a corner of a joint face) and a translation vector
         *  that indicates how much the joint face should be translated to better fit the skeleton and target area.
         */
        std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> _ComputeJointFaceTranslationsForOptimisation();

        std::vector<Joinery::JointGroup> _jointGroups;
        std::vector<Eigen::Vector3d> _skeleton;
        RoundwoodJoinery::PointCloud::PointCloud _pointCloud;
        double _referenceDiameter;
        /**
         * @brief A matrix that stores the relative degree of freedom between the joint groups, by using the indexes of the joint groups. It is symmetric and the diagonal is empty.
         */
        std::vector<std::vector<Eigen::Vector3d>> _relativeDegreesOfFreedom;
    };
}