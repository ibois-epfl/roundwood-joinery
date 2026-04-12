#pragma once

#include <string>

#include <Eigen/Dense>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>

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

            /**
             * @brief Projects points from the point cloud onto the joint face.
             * 
             * @param pointCloud The point cloud containing the points to be projected.
             * @return A vector of Eigen::Vector3d representing the projected points.
             */
            std::vector<Eigen::Vector3d> ProjectPointsOntoFace(RoundwoodJoinery::PointCloud::PointCloud& pointCloud);

            // Getters and small utils
            /**
            * @brief Returns the normal vector of the joint face.
            * @return The normal vector of the joint face.
            */
            Eigen::Vector3d GetNormal() const
            {
                return this->_normal;
            }

            /**
            * @brief Flips the normal vector of the joint face.
            */
            void FlipNormal()
            {
                this->_normal = -this->_normal;
                std::reverse(this->_corners.begin(), this->_corners.end());
            }

            std::vector<Eigen::Vector3d> GetCorners() const
            {
                return this->_corners;
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
            * @param pointCloud The point cloud of the beam to which the joint face belongs.
            * @param alpha The alpha parameter for the alpha shape computation, which is used to determine the outline of the projected points.
            * @return The computed current area of the joint face.
            */
            double ComputeCurrentArea(PointCloud::PointCloud& pointCloud, double alpha = 500.0);

            /**
            * @brief Returns the current outline of the joint face based on the points from the beam's point cloud that are projected onto the face.
            * 
            * @param pointCloud The point cloud of the beam to which the joint face belongs.
            * @param alpha The alpha parameter for the alpha shape computation, which is used to determine the outline of the projected points.
            * @return A vector of Eigen::Vector3d representing the current outline of the joint face.
            */
            std::vector<Eigen::Vector3d> GetCurrentOutline(PointCloud::PointCloud& pointCloud, double alpha = 500.0);


            void ApplyTransformation(Eigen::Matrix4d transformation);

        private:
            Eigen::Vector3d _normal;
            Eigen::Vector3d _center = Eigen::Vector3d::Zero();
            std::vector<Eigen::Vector3d> _corners;
            std::vector<Eigen::Vector3d> _originalCorners; // A backup should it be useful...
            double _targetArea;
            double _currentArea;
            std::vector<Eigen::Vector3d> _projectedPoints;
            
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
            Joint(std::vector<std::shared_ptr<JointFace>> faces);
            Joint() = default;
            ~Joint() = default;

            /**
            * @brief Returns the faces that make up the joint.
            * @return A vector of JointFace objects representing the faces of the joint.
            */
            std::vector<std::shared_ptr<JointFace>> GetFaces()
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

            /**
            * @brief A method to apply a transformation to the joint, which will be applied to all its faces.
            * @param transformation The 4x4 transformation matrix to be applied to the joint.
            */
            void ApplyTransformation(Eigen::Matrix4d transformation);

        private:
            std::vector<std::shared_ptr<JointFace>> _faces;
            Eigen::Vector3d _center = Eigen::Vector3d::Zero();
            Eigen::Vector3d _closestPointOnSkeleton = Eigen::Vector3d::Zero();
    };

    /**
     * @brief a class to store the joints by rigid groups 
     */
    class JointGroup
    {
        public:
            JointGroup(std::vector<std::shared_ptr<Joint>> joints) : _joints(joints) {};
            ~JointGroup() = default;

            /**
            * @brief Returns the joints that make up the joint group.
            * @return A vector of shared pointers to Joint objects representing the joints in the group.
            */
            std::vector<std::shared_ptr<Joint>> GetJoints()
            {
                return this->_joints;
            }

            void SetDegreeOfFreedom(Eigen::Vector3d degreeOfFreedom)
            {
                this->_degreeOfFreedom = degreeOfFreedom;
            }

            Eigen::Vector3d GetDegreeOfFreedom() const
            {
                return this->_degreeOfFreedom;
            }

            Eigen::Vector3d GetCentroid() const
            {
                Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                for (const auto& joint : this->_joints)
                {
                    centroid += joint->GetCenter();
                }
                return centroid / this->_joints.size();
            }

            void ApplyTransformation(Eigen::Matrix4d transformation)
            {
                for (auto& joint : this->_joints)
                {
                    joint->ApplyTransformation(transformation);
                }

                // also allow transform the dof.
                Eigen::Vector3d translation = transformation.block<3,1>(0,3);
                Eigen::Matrix3d rotation = transformation.block<3,3>(0,0);
                this->_degreeOfFreedom = rotation * this->_degreeOfFreedom;
            }
        private:
            std::vector<std::shared_ptr<Joint>> _joints;
            Eigen::Vector3d _degreeOfFreedom;
    };
}