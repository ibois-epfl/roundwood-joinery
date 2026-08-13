#pragma once
#include <iostream>
#include <fstream>
#include <vector>
// for eigen umeyama
#include <Eigen/Geometry>

// for alpha meshing
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>

//for alpha shape
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
 
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
 
#include <CGAL/algorithm.h>

// for skeletonization
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>

// for 2D polygon in 3D
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;

// for closest pair of points search
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_segment_primitive_3.h>
#include <CGAL/Segment_3.h>
#include <CGAL/squared_distance_3.h>
typedef CGAL::Simple_cartesian<double> PPKernel;
typedef PPKernel::Point_3 PPPoint_3;
typedef PPKernel::Segment_3 PPSegment_3;
typedef std::vector<PPSegment_3> PPSegmentVector;
typedef CGAL::AABB_segment_primitive_3<PPKernel, PPSegmentVector::const_iterator> PPPrimitive;
typedef CGAL::AABB_traits_3<PPKernel, PPPrimitive> PPTraits;
typedef CGAL::AABB_tree<PPTraits> PPTree;

// For boolean operations
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_with_holes_2.h>
typedef K::Point_2 Point_2;

//other stuff
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
using BK = CGAL::Exact_predicates_exact_constructions_kernel;
using BPoint2 = BK::Point_2;
using BPoly2 = CGAL::Polygon_2<BK>;
using BPolyWithHoles2 = CGAL::Polygon_with_holes_2<BK>;
using BSet2 = CGAL::Polygon_set_2<BK>;


// for Coherent Point Drift
#include <cpd/rigid.hpp>

#include "../PointCloud/PointCloud.hh"

namespace RoundwoodJoinery::Utils
{
    /** Computes the skeleton of a given point cloud.
     *  @param pointCloud The input point cloud.
     *  @param alpha The alpha parameter for the alpha shape algorithm. Controls the level of detail of the skeleton.
     *  @param offset The offset parameter for the alpha shape algorithm. Controls the distance from the original points to the skeleton.
     *
     *  @return A vector of 3D points representing the skeleton of the point cloud.
     */
    std::vector<Eigen::Vector3d> ComputePointCloudSkeleton(const PointCloud::PointCloud& pointCloud, double alpha, double offset);

    /**
     * @brief Computes the height of a triangle formed by a test point and a base defined by two points.
     * 
     * @param testPoint The point from which the height is measured.
     * @param baseStart The starting point of the base of the triangle.
     * @param baseEnd The ending point of the base of the triangle.
     * @return The point on the base that is closest to the test point, representing the height of the triangle.
     */
    Eigen::Vector3d FindHeightOfTriangle(Eigen::Vector3d testPoint, 
                                         Eigen::Vector3d baseStart,
                                         Eigen::Vector3d baseEnd);
    /**
     * @brief Computes the 2D alpha shape of a set of points that all lie on the same plane, using CGAL.
     * 
     * @param points The set of points lying on the same plane, no re-projection is computed nor planarity checked...
     * @param alpha The alpha parameter for the alpha shape algorithm. Controls the level of detail of the shape.
     * @param normal The normal vector of the plane on which the points lie.
     * @return A vector of 3D points representing the 2D alpha shape.
     */
    std::vector<Eigen::Vector3d> Compute2DAlphaShape(const std::vector<Eigen::Vector3d>& points, double alpha, Eigen::Vector3d normal);

    /**
     * @brief Saves a point cloud to a PLY file.
     * 
     * @param points The point cloud to be saved, represented as a vector of 3D points.
     * @param filename The name of the PLY file to save the point cloud to.
     */
    void SavePointCloudToPLY(const std::vector<Eigen::Vector3d>& points, const std::string& filename);

    /**
     * @brief Computes an approximating transformation matrix based on a set of anchor points and their corresponding translations.
     * 
     * @param groupedAnchorPointsAndTranslations A vector of vectors of pairs, where each inner vector corresponds to a group of anchor points,
     *  and each pair consists of an anchor point and its corresponding translation.
     * @return A vector of 4x4 transformation matrices for each group of joints.
     */
    std::vector<Eigen::Matrix4d> ComputeApproximatingTransformation(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> groupedAnchorPointsAndTranslations);


    /**
     * @brief Computes a collective approximating transformation matrix by averaging the transformations computed for each group of joints. It does the same as ComputeApproximatingTransformation but but without the per-joint granularity, and returns only one transformation.
     * 
     * @param groupedAnchorPointsAndTranslations A vector of vectors of pairs, where each inner vector corresponds to a group of anchor points,
     *  and each pair consists of an anchor point and its corresponding translation.
     * @return A single 4x4 transformation matrix representing the collective transformation for all groups of joints.
     */
    Eigen::Matrix4d ComputeCollectiveApproximatingTransformation(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> groupedAnchorPointsAndTranslations);
    
    /**
     * @brief Computes a transformation matrix that aligns a source curve to a target curve using coherent point drift. This methods allows for 2 sets of point of different size, and only partial overlap.
     * 
     * @param sourceCurve The source curve represented as a vector of 3D points.
     * @param targetCurve The target curve represented as a vector of 3D points.
     * @return A 4x4 transformation matrix that aligns the source curve to the target curve.
     */
    Eigen::Matrix4d ComputeCurveToCurveTransformation(const std::vector<Eigen::Vector3d>& sourceCurve, const std::vector<Eigen::Vector3d>& targetCurve);
    
    /**
     * @brief Computes a 2D polygon from a set of 3D points projected onto a plane defined by a normal vector.
     * 
     * @param points The set of 3D points to be projected onto the plane.
     * @param normal The normal vector of the plane onto which the points are projected.
     * @return A CGAL 2D polygon representing the projected points.
     */
    CGAL::Polygon_2<CGAL::Projection_traits_3<K>> Compute2DPolygon(std::vector<Eigen::Vector3d> points, Eigen::Vector3d normal);


    /**
     * @brief Computes a 2D polygon from a set of 3D points.
     * 
     * @param points The set of 3D points.
     * @param normal The normal vector of the plane.
     * @param planeOrigin A point on the plane onto which the points are projected.
     * 
     * @return A CGAL 2D polygon representing the projected points.
     */
    CGAL::Polygon_2<K> Compute2DPolygonInPlane(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& normal, const Eigen::Vector3d& planeOrigin);


    /**
     * @brief Builds an exact (with exact kernel) 2D polygon from a set of 3D points projected onto a plane defined by two direction vectors and a point on the plane.
     * 
     * @param outline The set of 3D points to be projected onto the plane.
     * @param yDirection The first direction vector defining the plane.
     * @param zDirection The second direction vector defining the plane.
     * @param pointOnSkeleton A point on the plane onto which the points are projected.
     * @return A CGAL exact 2D polygon representing the projected points.
     */
    BPoly2 BuildExact2DPolygon(const std::vector<Eigen::Vector3d>& outline,
                                    const Eigen::Vector3d& yDirection,
                                    const Eigen::Vector3d& zDirection,
                                    const Eigen::Vector3d& pointOnSkeleton);

    /**
     * @brief Checks if a given outline intersects a specified plane.
     * 
     * @param outline The outline represented as a vector of 3D points.
     * @param planePoint A point on the plane.
     * @param planeNormal The normal vector of the plane.
     * @return True if the outline crosses the plane, false otherwise.
     */
    bool IsOutlineIntersectingPlane(const std::vector<Eigen::Vector3d>& outline, const Eigen::Vector3d& planePoint, const Eigen::Vector3d& planeNormal);

    /**
     * @brief Computes the mean transformation from a set of transformation matrices by averaging their rotation (through quaternions) and translation components separately.
     * 
     * @param transformations The set of transformation matrices to average.
     * @return The mean transformation matrix.
     */
    Eigen::Matrix4d ComputeMeanTransformation(const std::vector<Eigen::Matrix4d>& transformations);

    /**
     * @brief Computes the closest points between two curves represented as vectors of 3D points.
     * 
     * @param curve1 The first curve represented as a vector of 3D points.
     * @param curve2 The second curve represented as a vector of 3D points.
     * @return A pair of 3D points representing the closest points on each curve.
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeClosestPointsBetweenTwoCurves(const std::vector<Eigen::Vector3d>& curve1, const std::vector<Eigen::Vector3d>& curve2);

    /**
     * @brief Projects a point onto a plane along a specified direction.
     * 
     * @param point The point to be projected.
     * @param planePoint A point on the plane onto which the point is projected.
     * @param planeNormal The normal vector of the plane.
     * @param direction The direction along which the point is projected onto the plane.
     * @return A pair containing the projected point on the plane and a boolean indicating whether the projection is valid.
     */
    std::pair<Eigen::Vector3d, bool> ProjectPointOnPlaneAlongDirection(const Eigen::Vector3d& point, const Eigen::Vector3d& planePoint, const Eigen::Vector3d& planeNormal, const Eigen::Vector3d& direction);
}