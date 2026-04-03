# pragma once

#include <cstdint>
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

        /**
         * @brief Constructs a PointCloud object with the given points.
         */
        PointCloud(std::vector<Eigen::Vector3d> points) : _points(points) {}

        ~PointCloud() = default;

        bool LoadFromFile(const std::string& filename);

        std::vector<Eigen::Vector3d> GetPoints() const
        {
            return this->_points;
        }

        std::vector<Eigen::Vector3d> Get1PcntPoints() const
        {
            size_t numPoints = this->_points.size();
            std::vector<Eigen::Vector3d> sampledPoints;
            for (size_t i = 0; i < numPoints; i += 100)
            {
                sampledPoints.push_back(this->_points[i]);
            }
            return sampledPoints;
        }

    private:
        std::vector<Eigen::Vector3d> _points;
    };
}