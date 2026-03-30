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
#include "../Utils/Utils.hh"

namespace RoundwoodJoinery::Joinery
{
    /**
     * @brief Represents a face of a joint, containing its normal vector, center point, and area.
     */
    class JointFace
    {
    public:
        JointFace(Eigen::Vector3d normal, std::vector<Eigen::Vector3d> corners, double targetArea = 0.0);
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
         * @return The target area of the joint face.
         */
        double GetTargetArea() const
        {
            return this->_targetArea;
        }

        /**
         * @brief Returns the current area of the joint face.
         * @return The current area of the joint face.
         */
        double GetCurrentArea() const
        {
            return this->_currentArea;
        }

        /**
         * @brief Computes the current area of the joint face based on the points from the beam's point cloud that are projected onto the face.
         * 
         * @param beamPointCloud The point cloud of the beam to which the joint face belongs.
         * @return The computed current area of the joint face.
         */
        double ComputeCurrentArea(PointCloud::PointCloud& beamPointCloud);

    private:
        Eigen::Vector3d _normal;
        Eigen::Vector3d _center = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> _corners;
        double _targetArea;
        double _currentArea;
        
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

        /**
         * @brief Returns the faces that make up the joint.
         * @return A vector of JointFace objects representing the faces of the joint.
         */
        std::vector<JointFace> GetFaces()
        {
            return this->_faces;
        }

        /**
         * @brief Returns the center point of the joint.
         * @return The center point of the joint.
         */
        Eigen::Vector3d GetCenter() const
        {
            return this->_center;
        }

        /**
         * @brief Returns the number of faces in the joint.
         * @return The number of faces in the joint.
         */
        size_t GetNumFaces()
        {
            return this->_faces.size();
        }

        void SetClosestPointOnSkeleton(Eigen::Vector3d correspondance)
        {
            this->_closestPointOnSkeleton = correspondance;
        }

        /**
         * @brief Returns the closest point on the skeleton to this joint. If the closest point has not been set, it returns (0,0,0) and prints a warning message.
         * @return The closest point on the skeleton to this joint.
         */
        Eigen::Vector3d GetClosestPointOnSkeleton() const
        {
            if (this->_closestPointOnSkeleton == Eigen::Vector3d::Zero())
            {
                std::cerr << "Warning: Closest point on skeleton has not been set for this joint. Returning (0,0,0) as default." << std::endl;
            }
            return this->_closestPointOnSkeleton;
        }

    private:
        std::vector<JointFace> _faces;
        Eigen::Vector3d _center = Eigen::Vector3d::Zero();
        Eigen::Vector3d _closestPointOnSkeleton = Eigen::Vector3d::Zero();
    };
}