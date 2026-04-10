# include "Joint.hh"

namespace RoundwoodJoinery::Joinery
{
    RoundwoodJoinery::Joinery::JointFace::JointFace(Eigen::Vector3d normal, std::vector<Eigen::Vector3d> corners, double targetArea)
        : _normal(normal), _corners(corners), _originalCorners(corners), _targetArea(targetArea)
    {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto& corner : corners){center += corner;}
        center /= corners.size();
        this->_center = center;

        this->_outline_polygon = std::move(Utils::Compute2DPolygon(corners, normal));
    }

    std::vector<Eigen::Vector3d> JointFace::ProjectPointsOntoFace(RoundwoodJoinery::PointCloud::PointCloud& pointCloud)
    {
        // First some basic data about joint face
        Eigen::Vector3d jointCenter = this->_center;
        Eigen::Vector3d firstCorner = this->_corners[0];
        Eigen::Vector3d farthestCorner = this->_corners[0];
        double maxDistance = 0.0;
        for (const auto& corner : this->_corners)
        {
            double distance = (corner - firstCorner).norm();
            if (distance > maxDistance)
            {
                maxDistance = distance;
                farthestCorner = corner;
            }
        }
        double sphereRadius = maxDistance / 2.0;

        //TODO : remove timing before release 0.1.0
        auto t1 = std::chrono::high_resolution_clock::now();
        Fuzzy_sphere fuzzySphere(Point(jointCenter.x(), jointCenter.y(), jointCenter.z()), sphereRadius);
        std::vector<Point> neighborhoodPoints;
        std::shared_ptr<CGAL::Kd_tree<Traits>> tree = pointCloud.BuildKdTree();
        tree->search(std::back_inserter(neighborhoodPoints), fuzzySphere);
        
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Octree query: " << std::chrono::duration<double>(t2-t1).count() << "s\n";
        std::vector<Eigen::Vector3d> projectedPoints;
        std::cout << "Number of points in the neighborhood: " << neighborhoodPoints.size() << std::endl;
        Eigen::Vector3d normal = this->_normal.normalized();
        for (const auto& point : neighborhoodPoints)
        {
            Eigen::Vector3d pointVec(point.x(), point.y(), point.z());
            double dist = (pointVec - this->_center).dot(normal);
            // if this dot product is negative, the point is "behind" the face and we should ignore it
            if (dist < 0){continue;}
            Eigen::Vector3d projection = pointVec - (dist * normal);
            CGAL::Projection_traits_3<K> traits({normal.x(), normal.y(), normal.z()});
            if (this->_outline_polygon) 
            {
                switch(CGAL::bounded_side_2(this->_outline_polygon->vertices_begin(), this->_outline_polygon->vertices_end(),
                                                Point_3(projection.x(), projection.y(), projection.z()),traits))
                {
                    case CGAL::ON_BOUNDED_SIDE:
                    case CGAL::ON_BOUNDARY:
                        projectedPoints.push_back(projection);
                        break;
                    default:
                        break;
                }
            }
            else
            {
                std::cerr << "Warning: No outline polygon defined for this face. All projected points will be included." << std::endl;
                projectedPoints.push_back(projection);
            }
        }
        this->_projectedPoints = projectedPoints;
        return projectedPoints;
    }

    double RoundwoodJoinery::Joinery::JointFace::ComputeCurrentArea(RoundwoodJoinery::PointCloud::PointCloud& pointCloud, double alpha)
    {
        if (this->_projectedPoints.empty())
        {
            this->_projectedPoints = this->ProjectPointsOntoFace(pointCloud);
        }

        if (this->_projectedPoints.size() < 3)
        {
            std::cerr << "Warning: Not enough points projected onto the face to compute area. Returning 0." << std::endl;
            return 0.0;
        }
        if (this->_normal == Eigen::Vector3d::Zero())
        {
            std::cerr << "Warning: Normal vector is zero. Cannot compute area. Returning 0." << std::endl;
            return 0.0;
        }
        
        std::vector<Eigen::Vector3d> alphaShapePoints = Utils::Compute2DAlphaShape(this->_projectedPoints, alpha, this->_normal);
        // Compute the area of the alpha shape polygon
        CGAL::Projection_traits_3<K> traits({this->_normal.x(), this->_normal.y(), this->_normal.z()});
        CGAL::Polygon_2<CGAL::Projection_traits_3<K>> cgalPolygon = Utils::Compute2DPolygon(alphaShapePoints, this->_normal);

        cgalPolygon.reverse_orientation();
        this->_currentArea = cgalPolygon.area();

        return this->_currentArea;
    }

    std::vector<Eigen::Vector3d> RoundwoodJoinery::Joinery::JointFace::GetCurrentOutline(RoundwoodJoinery::PointCloud::PointCloud& pointCloud, double alpha)
    {
        if (this->_projectedPoints.empty())
        {
            this->_projectedPoints = this->ProjectPointsOntoFace(pointCloud);
        }

        if (this->_projectedPoints.size() < 3)
        {
            std::cerr << "Warning: Not enough points projected onto the face to compute outline. Returning empty vector." << std::endl;
            return {};
        }

        std::vector<Eigen::Vector3d> alphaShapePoints = Utils::Compute2DAlphaShape(this->_projectedPoints, alpha, this->_normal);

        return alphaShapePoints;
    }

    void RoundwoodJoinery::Joinery::JointFace::ApplyTransformation(Eigen::Matrix4d transformation)
    {
        Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
        Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);
        this->_normal = rotation * this->_normal;
        this->_center = rotation * this->_center + translation;

        for (size_t i = 0; i < this->_corners.size(); ++i)
        {
            this->_corners[i] = rotation * this->_corners[i] + translation;
        }

        this->_outline_polygon = std::move(Utils::Compute2DPolygon(this->_corners, this->_normal));
        this->_projectedPoints.clear();
        this->_currentArea = 0.0;
    }

    RoundwoodJoinery::Joinery::Joint::Joint(std::vector<RoundwoodJoinery::Joinery::JointFace> faces)
        : _faces(faces)
    {
        for (const auto& face : faces)
        {
            this->_center += face.GetCenter();
        }
        this->_center /= faces.size();

    }

    void RoundwoodJoinery::Joinery::Joint::ApplyTransformation(Eigen::Matrix4d transformation)
    {
        // first the Joint data
        Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
        Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);
        this->_center = rotation * this->_center + translation;

        // then the JointFaces
        for (auto& face : this->_faces)
        {
            face.ApplyTransformation(transformation);
        }
    }
}