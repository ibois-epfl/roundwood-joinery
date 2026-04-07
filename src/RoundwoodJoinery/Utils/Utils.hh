#pragma once

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
     * @brief Computes a 2D polygon from a set of 3D points projected onto a plane defined by a normal vector.
     * 
     * @param points The set of 3D points to be projected onto the plane.
     * @param normal The normal vector of the plane onto which the points are projected.
     * @return A CGAL 2D polygon representing the projected points.
     */
    CGAL::Polygon_2<CGAL::Projection_traits_3<K>> Compute2DPolygon(std::vector<Eigen::Vector3d> points, Eigen::Vector3d normal);
}