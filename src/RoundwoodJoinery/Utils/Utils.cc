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
        };

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

    std::vector<Eigen::Vector3d> Compute2DAlphaShape(const std::vector<Eigen::Vector3d>& points, double alpha, Eigen::Vector3d normal)
    {
        std::vector<Eigen::Vector2d> verticesInPlane;
        for (const auto& point : points)
        {
            verticesInPlane.emplace_back(point.x(), point.y());
        }
        typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
        typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
        typedef CGAL::Alpha_shape_face_base_2<K> Fb;
        typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
        typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;
        typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;

        std::vector<K::Point_2> cgalPoints2D;
        for (size_t i = 0; i < verticesInPlane.size(); ++i)
        {
            cgalPoints2D.emplace_back(verticesInPlane[i].x(), verticesInPlane[i].y());
        }

        Alpha_shape_2 A(cgalPoints2D.begin(), cgalPoints2D.end(), alpha, Alpha_shape_2::REGULARIZED);
        std::vector<Eigen::Vector3d> alphaShapePoints;
        
        std::map<std::pair<double, double>, std::vector<std::pair<double, double>>> adjacency;
        for (auto it = A.finite_edges_begin(); it != A.finite_edges_end(); ++it) 
        {
            if (A.classify(*it) == Alpha_shape_2::REGULAR) 
            {
                auto seg = A.segment(*it);
                std::pair<double, double> p1 = {seg.source().x(), seg.source().y()};
                std::pair<double, double> p2 = {seg.target().x(), seg.target().y()};
                adjacency[p1].push_back(p2);
                adjacency[p2].push_back(p1);
            }
        }

        std::pair<double, double> start = adjacency.begin()->first;
        std::vector<std::pair<double, double>> ordered2D;
        std::set<std::pair<double, double>> visited;
        auto current = start;
        std::pair<double, double> prev = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
        while (true) 
        {
            ordered2D.push_back(current);
            visited.insert(current);
            // Find the next neighbor that is not the previous point
            const auto& neighbors = adjacency[current];
            std::pair<double, double> next;
            if (neighbors.size() == 1) 
            {
                next = neighbors[0];
            }
            else if (neighbors.size() == 2)
            {
                next = (neighbors[0] == prev) ? neighbors[1] : neighbors[0];
            }
            if (visited.count(next)) break;
            prev = current;
            current = next;
        }
        // imperfect way to reintroduce the z coordinate.
        // We rely on the fact that the alpha shape points are a subset of the original points, 
        // so we can find the corresponding z value in the original point cloud
        std::vector<Eigen::Vector3d> ordered3D;
        for (const auto& p2d : ordered2D) 
        {
            for (const auto& p3d : points) 
            {
                if (std::abs(p3d.x() - p2d.first) < 1e-6 && std::abs(p3d.y() - p2d.second) < 1e-6) 
                {
                    ordered3D.push_back(p3d);
                    break;
                }
            }
        }
        return ordered3D;
    }

    void SavePointCloudToPLY(const std::vector<Eigen::Vector3d>& points, const std::string& filename)
    {
        std::vector<std::array<double, 3>> meshVertexPositions;
        for (const auto& p : points)
        {
            meshVertexPositions.push_back({p.x(), p.y(), p.z()});
        }

        happly::PLYData plyOut;
        plyOut.addVertexPositions(meshVertexPositions);
        plyOut.write(filename, happly::DataFormat::ASCII);
    }

    std::vector<Eigen::Matrix4d> ComputeApproximatingTransformation(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> groupedAnchorPointsAndTranslations)
    {
        std::vector<Eigen::Matrix4d> transformations;
        for (const auto& anchorPointsAndTranslations : groupedAnchorPointsAndTranslations)
        {
            Eigen::MatrixXd sourcePoints(3, anchorPointsAndTranslations.size());
            Eigen::MatrixXd targetPoints(3, anchorPointsAndTranslations.size());

            for (size_t i = 0; i < anchorPointsAndTranslations.size(); ++i)
            {
                const auto& pair = anchorPointsAndTranslations[i];
                sourcePoints.col(i) = pair.first;
                targetPoints.col(i) = pair.first + pair.second;
            }

            Eigen::Matrix4d transformation = Eigen::umeyama(sourcePoints, targetPoints, false);
            transformations.push_back(transformation);
        }
        return transformations;
    }

    CGAL::Polygon_2<CGAL::Projection_traits_3<K>> Compute2DPolygon(std::vector<Eigen::Vector3d> points, Eigen::Vector3d normal)
    {
        CGAL::Projection_traits_3<K> traits({normal.x(), normal.y(), normal.z()});
        CGAL::Polygon_2<CGAL::Projection_traits_3<K>> cgalPolygon(traits);
        for (const auto& point : points)
        {
            cgalPolygon.push_back(Point_3(point.x(), point.y(), point.z()));
        }
        return cgalPolygon;
    }
}