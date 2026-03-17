#pragma once

#include <string>

#include <Eigen/Dense>

namespace RoundwoodJoinery::Joinery
{
    /**
     * @brief Represents a face of a joint, containing its normal vector, center point, and area.
     */
    class JointFace
    {
    public:
        JointFace(Eigen::Vector3d normal, Eigen::Vector3d center, double area);
        ~JointFace() = default;

        // Getters
        /**
         * @brief Returns the normal vector of the joint face.
         * @return The normal vector of the joint face.
         */
        Eigen::Vector3d GetNormal() const
        {
            return this->normal;
        }

        /**
         * @brief Returns the center point of the joint face.
         * @return The center point of the joint face.
         */
        Eigen::Vector3d GetCenter() const
        {
            return this->center;
        }

        /**
         * @brief Returns the area of the joint face.
         * @return The area of the joint face.
         */
        double GetArea() const
        {
            return this->_area;
        }
    
    private:
        Eigen::Vector3d normal;
        Eigen::Vector3d center;
        double _area;
    };

    /**
     * @brief Represents a joint, which consists of multiple faces. Each joint has a specific type that can be retrieved using the getType() method.
     */
    class Joint
    {
    public:
        Joint(std::vector<JointFace> faces);
        Joint() = default;
        ~Joint() = default;

        std::vector<JointFace> GetFaces();

        /**
         * @brief Returns the number of faces in the joint.
          * @return The number of faces in the joint.
         */
        size_t GetNumFaces()
        {
            return this->_faces.size();
        }
    
    private:
        std::vector<JointFace> _faces;
    };
}