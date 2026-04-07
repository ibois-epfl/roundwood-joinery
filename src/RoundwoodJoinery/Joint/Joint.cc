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

        std::vector<Point_3> cgal_corners;
        for (const auto& corner : corners)
        {
            cgal_corners.emplace_back(Point_3(corner.x(), corner.y(), corner.z()));
        }
        CGAL::Projection_traits_3<K> traits({normal.x(), normal.y(), normal.z()});
        CGAL::Polygon_2<CGAL::Projection_traits_3<K>> cgalPolygon(traits);
        for (const auto& corner : corners) 
        {
            cgalPolygon.push_back(Point_3(corner.x(), corner.y(), corner.z()));
        }
        this->_outline_polygon = std::move(cgalPolygon);
    }

    std::vector<Eigen::Vector3d> JointFace::ProjectPointsOntoFace(RoundwoodJoinery::PointCloud::PointCloud& pointCloud)
    {
        std::vector<Eigen::Vector3d> projectedPoints;
        Eigen::Vector3d normal = this->_normal.normalized();
        for (const auto& point : pointCloud.GetPoints())
        {
            double dist = (point - this->_center).dot(normal);
            // if this dot product is negative, the point is "behind" the face and we should ignore it
            if (dist < 0){continue;}
            Eigen::Vector3d projection = point - (dist * normal);
            CGAL::Projection_traits_3<K> traits({normal.x(), normal.y(), normal.z()});
            if (_outline_polygon) 
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

    double RoundwoodJoinery::Joinery::JointFace::ComputeCurrentArea(PointCloud::PointCloud& beamPointCloud, double alpha)
    {
        if (this->_projectedPoints.empty())
        {
            this->_projectedPoints = this->ProjectPointsOntoFace(beamPointCloud);
        }

        if (this->_projectedPoints.size() < 3)
        {
            std::cerr << "Warning: Not enough points projected onto the face to compute area. Returning 0." << std::endl;
            return 0.0;
        }
        
        std::vector<Eigen::Vector3d> alphaShapePoints = Utils::Compute2DAlphaShape(this->_projectedPoints, alpha, this->_normal);
        // Compute the area of the alpha shape polygon
        CGAL::Projection_traits_3<K> traits({this->_normal.x(), this->_normal.y(), this->_normal.z()});
        CGAL::Polygon_2<CGAL::Projection_traits_3<K>> cgalPolygon(traits);
        for (const auto& point : alphaShapePoints)
        {
            cgalPolygon.push_back(Point_3(point.x(), point.y(), point.z()));
        }

        cgalPolygon.reverse_orientation();
        this->_currentArea = cgalPolygon.area();

        return this->_currentArea;
    }

    std::vector<Eigen::Vector3d> RoundwoodJoinery::Joinery::JointFace::GetCurrentOutline(PointCloud::PointCloud& beamPointCloud, double alpha)
    {
        if (this->_projectedPoints.empty())
        {
            this->_projectedPoints = this->ProjectPointsOntoFace(beamPointCloud);
        }

        if (this->_projectedPoints.size() < 3)
        {
            std::cerr << "Warning: Not enough points projected onto the face to compute outline. Returning empty vector." << std::endl;
            return {};
        }

        std::vector<Eigen::Vector3d> alphaShapePoints = Utils::Compute2DAlphaShape(this->_projectedPoints, alpha, this->_normal);

        return alphaShapePoints;
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
}