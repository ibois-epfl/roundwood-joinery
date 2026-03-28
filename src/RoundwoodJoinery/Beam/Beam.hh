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
        Beam(std::vector<std::shared_ptr<Joinery::Joint>> joints, std::vector<Eigen::Vector3d> skeleton);
        ~Beam() = default;
        std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::Joint>> GetJoints() const
        {
            return this->_joints;
        }
        std::vector<Eigen::Vector3d> GetSkeleton() const
        {
            return this->_skeleton;
        }
    private:
        std::vector<std::shared_ptr<Joinery::Joint>> _joints;
        std::vector<Eigen::Vector3d> _skeleton;
    };
}