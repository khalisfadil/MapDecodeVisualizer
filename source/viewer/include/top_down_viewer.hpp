// MIT License

// Copyright (c) 2024 Muhammad Khalis bin Mohd Fadil

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/visualization/visualizer/Visualizer.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <tsl/robin_map.h>

#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

// -----------------------------------------------------------------------------
/**
 * @brief A class to manage top-down visualization of points and a vehicle in NED frame.
 */
class TopDownViewer {
public:

    // -----------------------------------------------------------------------------
    /**
     * @brief Creates a 2D top-down voxel-based visualization mesh using the brightest voxel information.
     *
     * This function generates a mesh of square-shaped voxels based on a set of input 3D points and their 
     * corresponding grayscale intensity values. Each voxel represents the brightest point in a grid cell
     * (based on XY coordinates), and its brightness is determined by the provided grayscale values.
     * The output mesh includes colored squares at the Z = 0 plane, representing the voxels.
     *
     * @param points A vector of 3D points in world coordinates (Eigen::Vector3f).
     * @param vehicle_position The 3D position of the vehicle, used as the origin for relative calculations (Eigen::Vector3f).
     * @param grayscale_values A vector of grayscale intensity values (Eigen::Vector3i) for each point.
     *                         The x-component of Eigen::Vector3i represents the intensity.
     * @param mapRes The resolution of the voxel grid, specifying the size of each voxel.
     * 
     * @return A shared pointer to an Open3D TriangleMesh representing the voxel-based 2D map.
     *
     * @throws std::invalid_argument if the size of `points` and `grayscale_values` vectors do not match.
     *
     * @note The Z-coordinate of the output mesh is fixed at 0. The grayscale intensity values are normalized
     *       to the range [0, 1] for use as vertex colors.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVoxelSquares(
        const std::vector<Eigen::Vector3f>& points,
        const Eigen::Vector3f& vehicle_position,
        const std::vector<Eigen::Vector3i>& grayscale_values,
        float mapRes);

    // -----------------------------------------------------------------------------
    /**
     * @brief Creates a triangular mesh representing a vehicle with a specific size and orientation.
     *
     * The vehicle is represented as a triangular marker in the XY plane, with its orientation defined
     * by the yaw angle (in radians). The marker is green by default.
     *
     * @param markersize The size of the vehicle marker (triangle side length). Must be positive.
     * @param yaw_rad The yaw angle of the vehicle in radians.
     * 
     * @return A shared pointer to an Open3D TriangleMesh representing the vehicle.
     *
     * @throws std::invalid_argument if `markersize` is non-positive.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVehicleMesh(float markersize, double yaw_rad);

    // -----------------------------------------------------------------------------
    /**
     * @brief Creates a circle mesh to represent the vehicle.
     * 
     * @param radius The radius of the circle.
     * @param segments The number of segments to approximate the circle.
     * @return std::shared_ptr<open3d::geometry::TriangleMesh> The circular mesh.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVehicleCircleMesh(float radius, int segments);

    // -----------------------------------------------------------------------------
    /**
     * @brief Generate voxel cubes with colors based on grayscale values.
     * 
     * @param points Vector of points in NED frame.
     * @param vehicle_position Vehicle position in NED frame.
     * @param grayscale_values Grayscale values corresponding to each point.
     * @param mapRes Resolution of the map (size of a voxel).
     * @return std::shared_ptr<open3d::geometry::TriangleMesh> Mesh containing voxel cubes.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVoxelCubes(const std::vector<Eigen::Vector3f>& points,
                                                                        const Eigen::Vector3f& vehicle_position,
                                                                        const std::vector<Eigen::Vector3i>& grayscale_values, // Grayscale values per point
                                                                        float mapRes
                                                                        );

    // -----------------------------------------------------------------------------
    /**
     * @brief Creates a sphere mesh centered at the origin with a specified radius and smoothness.
     * 
     * This function generates a sphere mesh by creating vertices and triangles based on latitude 
     * and longitude segments. The sphere is colored uniformly, with each vertex assigned a green color.
     * 
     * @param radius The radius of the sphere. Must be a positive value.
     * @param latitude_segments The number of divisions along the vertical axis (latitude lines).
     *                          Higher values result in a smoother sphere.
     *                          Must be greater than or equal to 2.
     * @param longitude_segments The number of divisions around the horizontal axis (longitude lines).
     *                           Higher values result in a smoother sphere.
     *                           Must be greater than or equal to 3.
     * 
     * @return std::shared_ptr<open3d::geometry::TriangleMesh> 
     *         A shared pointer to the generated sphere mesh, ready for visualization in Open3D.
     * 
     * @throws std::invalid_argument If the radius is non-positive or if latitude/longitude segments are invalid.
     * 
     * @note The sphere is created in the NED (North-East-Down) coordinate frame, 
     *       and its center is placed at (0, 0, 0). Increasing the segment values improves mesh smoothness 
     *       at the cost of higher vertex and triangle count.
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateVehicleSphereMesh(float radius, int latitude_segments, int longitude_segments);
};
