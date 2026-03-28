#include "PointCloud.hh"

namespace RoundwoodJoinery::PointCloud
{
    bool PointCloud::LoadFromFile(const std::string& filename)
    {
        try
        {
            happly::PLYData plyIn(filename, true);
            auto vertices = plyIn.getVertexPositions();
            this->points.clear();
            for (const auto& vertex : vertices)
            {
                this->points.emplace_back(vertex[0], vertex[1], vertex[2]);
            }
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error loading point cloud from file: " << e.what() << std::endl;
            return false;
        }
    }
}