#include "Utils.hh"

namespace RoundwoodJoinery::Utils
{
    std::vector<Eigen::Vector3d> ComputePointCloudSkeleton(const PointCloud::PointCloud& pointCloud, double alpha, double offset)
    {
        using K = CGAL::Exact_predicates_inexact_constructions_kernel;
        using Point_3 = K::Point_3;
        using Point_container = std::vector<Point_3>;
        using Mesh = CGAL::Surface_mesh<Point_3>;

        std::vector<Eigen::Vector3d> pointsVec = pointCloud.Get1PcntPoints();

        Point_container points;
        for (const auto& p : pointsVec)
        {
            points.emplace_back(p[0], p[1], p[2]);
        }

        Mesh wrap;
        CGAL::alpha_wrap_3(points, alpha, offset, wrap);
        std::cout << "Result: " << num_vertices(wrap) << " vertices, " << num_faces(wrap) << " faces" << std::endl;
        CGAL::IO::write_polygon_mesh("output.ply", wrap, CGAL::parameters::stream_precision(17));

        CGAL::Mean_curvature_flow_skeletonization<Mesh>::Skeleton cgalSkeletonGraph;
 
        CGAL::extract_mean_curvature_flow_skeleton(wrap, cgalSkeletonGraph);
        auto vertice = cgalSkeletonGraph[0].point;
        std::cout << "Skeleton has " << vertice << " as first vertex." << std::endl;
        std::cout << "Skeleton has " << boost::num_vertices(cgalSkeletonGraph) << " vertices." << std::endl;
        std::vector<Eigen::Vector3d> skeleton = std::vector<Eigen::Vector3d>();
        auto point = cgalSkeletonGraph[0].point;
        skeleton.emplace_back(point[0], point[1], point[2]);
        std::vector<std::array<double, 3>> meshVertexPositions;

        for (int v = 1; v < boost::num_vertices(cgalSkeletonGraph); v++)
        {
            auto point = cgalSkeletonGraph[v].point;
            // sorting along x coordinates
            for (int i = 0; i < skeleton.size(); i++)
            {
                if (point[0] < skeleton[i][0])
                {
                    skeleton.insert(skeleton.begin() + i, Eigen::Vector3d(point[0], point[1], point[2]));
                    break;
                }
                if (i == skeleton.size() - 1)
                {
                    skeleton.emplace_back(point[0], point[1], point[2]);
                    break;
                }
            }
        }
        for (const auto& p : skeleton)
        {
            meshVertexPositions.push_back({p[0], p[1], p[2]});
        }

        happly::PLYData plyOut;
        plyOut.addVertexPositions(meshVertexPositions);
        plyOut.write("my_output_skeleton_file.ply", happly::DataFormat::ASCII);

        return skeleton;
    }

    Eigen::Vector3d FindHeightOfTriangle(Eigen::Vector3d testPoint, 
                                         Eigen::Vector3d baseStart,
                                         Eigen::Vector3d baseEnd)
    {
        Eigen::Vector3d baseVector = baseEnd - baseStart;
        Eigen::Vector3d testVector = testPoint - baseStart;

        double t = testVector.dot(baseVector) / baseVector.dot(baseVector);
        t = std::max(0.0, std::min(1.0, t)); // Clamp t to the range [0, 1]

        Eigen::Vector3d closestPointOnBase = baseStart + t * baseVector;
        return closestPointOnBase;
    }
}