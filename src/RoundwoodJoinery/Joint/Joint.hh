#pragma once

#include <string>

#include <Eigen/Dense>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
 
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;

#include "../PointCloud/PointCloud.hh"

namespace RoundwoodJoinery::Joinery
{
    /**
     * @brief Represents a face of a joint, containing its normal vector, center point, and area.
     */
    class JointFace
    {
    public:
        JointFace(Eigen::Vector3d normal, std::vector<Eigen::Vector3d> corners);
        ~JointFace() = default;

        std::vector<Eigen::Vector3d> ProjectPointsOntoFace(RoundwoodJoinery::PointCloud::PointCloud& pointCloud);

        // Getters
        /**
         * @brief Returns the normal vector of the joint face.
         * @return The normal vector of the joint face.
         */
        Eigen::Vector3d GetNormal() const
        {
            return this->_normal;
        }

        /**
         * @brief Returns the center point of the joint face.
         * @return The center point of the joint face.
         */
        Eigen::Vector3d GetCenter() const
        {
            return this->_center;
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
        Eigen::Vector3d _normal;
        Eigen::Vector3d _center;
        std::vector<Eigen::Vector3d> _corners;
        double _area = 0.0;
        
        /**
         * @brief the outline polygon that is optional for technical reasons, but is systematically created at construction
         */
        std::optional<CGAL::Polygon_2<CGAL::Projection_traits_3<K>>> _outline_polygon;
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