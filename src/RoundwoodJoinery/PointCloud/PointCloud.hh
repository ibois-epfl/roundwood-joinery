# pragma once

#include "../../3rd_party/happly/happly.h"
#include "../../3rd_party/eigen/Eigen/Dense"

namespace RoundwoodJoinery::PointCloud
{
    /**
     * @brief Represents a point cloud in a minimal manner
     */
    class PointCloud
    {
    public:
        PointCloud() = default;
        ~PointCloud() = default;

        bool LoadFromFile(const std::string& filename);

        std::vector<Eigen::Vector3d> GetPoints() const
        {
            return this->points;
        }

        std::vector<Eigen::Vector3d> Get1PcntPoints() const
        {
            size_t numPoints = this->points.size();
            std::vector<Eigen::Vector3d> sampledPoints;
            for (size_t i = 0; i < numPoints; i += 100)
            {
                sampledPoints.push_back(this->points[i]);
            }
            return sampledPoints;
        }

    private:
        std::vector<Eigen::Vector3d> points;
    };
}