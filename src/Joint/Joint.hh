#pragma once

#include <string>

#include <Eigen/Dense>

namespace RoundwoodJoinery
{
    class JointFace
    {
    public:
        JointFace(Eigen::Vector3d normal, Eigen::Vector3d center, double area);
        virtual ~JointFace() = default;
        virtual std::string getType() const = 0;
    
    private:
        Eigen::Vector3d normal;
        Eigen::Vector3d center;
        double area;
    };

    class Joint
    {
    public:
        Joint(std::vector<JointFace> faces);
        virtual ~Joint() = default;
        virtual std::string getType() const = 0;
    
    private:
        std::vector<JointFace> faces;
    };
}