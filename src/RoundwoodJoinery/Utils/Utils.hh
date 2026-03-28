#pragma once

// for alpha meshing
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
// for skeletonization
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>

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
}