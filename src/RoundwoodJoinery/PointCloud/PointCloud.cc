#include "PointCloud.hh"

namespace RoundwoodJoinery::PointCloud
{
    bool PointCloud::LoadFromFile(const std::string& filename)
    {
        try
        {
            happly::PLYData plyIn(filename, true);
            auto vertices = plyIn.getVertexPositions();
            this->_points.clear();
            for (const auto& vertex : vertices)
            {
                this->_points.emplace_back(vertex[0], vertex[1], vertex[2]);
            }
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error loading point cloud from file: " << e.what() << std::endl;
            return false;
        }
    }

    std::shared_ptr<CGAL::Kd_tree<Traits>> PointCloud::BuildKdTree()
    {
        std::list<Point> cgalPoints;
        for (const auto& point : this->_points)
        {
            cgalPoints.push_back(Point(point.x(), point.y(), point.z()));
        }
        auto tree = std::make_shared<CGAL::Kd_tree<Traits>>(cgalPoints.begin(), cgalPoints.end());
        this->_kdTree = tree;
        return tree;
    }
}