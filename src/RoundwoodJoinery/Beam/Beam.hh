#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "../Joint/Joint.hh"

namespace RoundwoodJoinery::Beam
{
    class Beam
    {
    public:
        Beam(std::vector<std::shared_ptr<Joinery::Joint>> joints, std::vector<Eigen::Vector3d> skeleton, RoundwoodJoinery::PointCloud::PointCloud pointCloud);
        ~Beam() = default;

        /**
         * @brief Returns the joints associated with the beam.
         * 
         * @return A vector of shared pointers to the joints associated with the beam.
         */
        std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::Joint>> GetJoints() const
        {
            return this->_joints;
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

    private:
        std::vector<std::shared_ptr<Joinery::Joint>> _joints;
        std::vector<Eigen::Vector3d> _skeleton;
        RoundwoodJoinery::PointCloud::PointCloud _pointCloud;
    };
}