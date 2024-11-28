#pragma once

#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <tsl/robin_map.h>

#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>

/**
 * @brief A class to manage top-down visualization of points and a vehicle in NED frame.
 */
class TopDownViewer {
public:

    /**
     * @brief Translate points to vehicle center and create top-down voxel squares.
     *
     * @param points Input points in NED (X, Y, Z) representing voxel centers.
     * @param vehicle_position Vehicle position in NED (X, Y, Z) to center the visualization.
     * @param map_resolution Size of each square voxel (length of each side in meters).
     * @param colors A vector of RGB colors for each voxel, normalized to [0, 1].
     * @return A shared pointer to an Open3D TriangleMesh containing the translated voxel squares.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVoxelSquares(const std::vector<Eigen::Vector3f>& points,
                                                                                    const Eigen::Vector3f& vehicle_position,
                                                                                    const std::vector<Eigen::Matrix<uint32_t, 3, 1>>& grayscale_values,
                                                                                    float map_res);

    /**
     * @brief Create a triangular representation of the vehicle aligned with its yaw.
     *
     * @param size The size of the triangle (e.g., vehicle length in meters).
     * @param yaw The yaw angle of the vehicle in degrees.
     * @return A shared pointer to the Open3D TriangleMesh representing the vehicle.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVehicleMesh(float markersize, double yaw_rad);


};
