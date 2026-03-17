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

    private:
        std::vector<Eigen::Vector3d> points;
    };
}