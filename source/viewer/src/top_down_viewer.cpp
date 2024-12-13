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
#include "top_down_viewer.hpp"

// -----------------------------------------------------------------------------
// Section: CreateVoxelSquares
// -----------------------------------------------------------------------------

std::shared_ptr<open3d::geometry::TriangleMesh> TopDownViewer::CreateVoxelSquares(
    const std::vector<Eigen::Vector3f>& points,
    const Eigen::Vector3f& vehicle_position,
    const std::vector<Eigen::Vector3i>& grayscale_values,
    float mapRes) 
{
    if (points.size() != grayscale_values.size()) {
        throw std::invalid_argument("Points and grayscale_values must have the same size.");
    }

    // Data structure to hold the brightest voxel for each unique XY key
    struct VoxelInfo {
        Eigen::Vector3i gridIndex; // Voxel grid index
        int grayscale;             // Grayscale value
    };

    // Function to generate a unique key for XY indices
    auto generateKey = [](int x, int y) -> int64_t {
        return (static_cast<int64_t>(x) << 32) | (static_cast<uint32_t>(y));
    };

    // Function for merging two maps
    auto merge_maps = [](tsl::robin_map<int64_t, VoxelInfo>& a, const tsl::robin_map<int64_t, VoxelInfo>& b) {
        for (const auto& [key, voxel_info] : b) {
            auto it = a.find(key);
            if (it == a.end() || voxel_info.grayscale > it->second.grayscale) {
                a[key] = voxel_info;
            }
        }
    };

    // Perform parallel_reduce to compute the brightest voxel per unique XY key
    tsl::robin_map<int64_t, VoxelInfo> brightest_voxels = tbb::parallel_reduce(
        tbb::blocked_range<uint64_t>(0, points.size()),
        tsl::robin_map<int64_t, VoxelInfo>(), // Initial empty map
        [&](const tbb::blocked_range<uint64_t>& range, tsl::robin_map<int64_t, VoxelInfo> local_map) -> tsl::robin_map<int64_t, VoxelInfo> {
            for (uint64_t i = range.begin(); i < range.end(); ++i) {
                // Translate point to vehicle-relative coordinates
                Eigen::Vector3f translated_point = points[i] - vehicle_position;

                // Convert translated point to grid index
                Eigen::Vector3f scaledPos = translated_point * (1.0f / mapRes);
                Eigen::Vector3i gridIndex(
                    static_cast<int>(std::floor(scaledPos.x())),
                    static_cast<int>(std::floor(scaledPos.y())),
                    static_cast<int>(std::floor(scaledPos.z()))
                );

                // Generate a unique key for the XY grid index
                int64_t key = generateKey(gridIndex.x(), gridIndex.y());

                // Update the local map
                auto it = local_map.find(key);
                int grayscale_value = grayscale_values[i].x(); // Access grayscale value
                if (it == local_map.end() || grayscale_value > it->second.grayscale) {
                    local_map[key] = VoxelInfo{gridIndex, grayscale_value};
                }
            }
            return local_map;
        },
        [&](tsl::robin_map<int64_t, VoxelInfo> left, const tsl::robin_map<int64_t, VoxelInfo>& right) {
            merge_maps(left, right);
            return left;
        }
    );

    // Create a triangle mesh for the selected voxels
    auto voxel_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    voxel_mesh->vertices_.reserve(brightest_voxels.size() * 4);
    voxel_mesh->triangles_.reserve(brightest_voxels.size() * 2);

    for (const auto& [key, voxel_info] : brightest_voxels) {
        // Convert grid index back to position
        Eigen::Vector3f voxelPos = voxel_info.gridIndex.cast<float>() * mapRes;

        // Compute square corners based on the voxel's X, Y position
        float half_size = mapRes / 2.0f;
        std::vector<Eigen::Vector3d> corners = {
            {voxelPos.x() - half_size, voxelPos.y() - half_size, 0.0}, // Bottom-left
            {voxelPos.x() + half_size, voxelPos.y() - half_size, 0.0}, // Bottom-right
            {voxelPos.x() + half_size, voxelPos.y() + half_size, 0.0}, // Top-right
            {voxelPos.x() - half_size, voxelPos.y() + half_size, 0.0}  // Top-left
        };

        // Assign color (normalize grayscale to [0, 1])
        Eigen::Vector3d color = Eigen::Vector3d::Constant(voxel_info.grayscale / 255.0);

        // Add vertices
        uint64_t base_index = voxel_mesh->vertices_.size();
        voxel_mesh->vertices_.insert(voxel_mesh->vertices_.end(), corners.begin(), corners.end());
        voxel_mesh->vertex_colors_.insert(voxel_mesh->vertex_colors_.end(), 4, color);

        // Add triangles
        voxel_mesh->triangles_.emplace_back(base_index, base_index + 1, base_index + 2); // Bottom-right triangle
        voxel_mesh->triangles_.emplace_back(base_index, base_index + 2, base_index + 3); // Top-left triangle
    }

    return voxel_mesh;
}

// -----------------------------------------------------------------------------
// Section: CreateVehicleMesh
// -----------------------------------------------------------------------------

std::shared_ptr<open3d::geometry::TriangleMesh> TopDownViewer::CreateVehicleMesh(float markersize, double yaw_ned) {
    if (markersize <= 0) {
        throw std::invalid_argument("Marker size must be positive.");
    }

    // Define a scaling factor for the triangle size
    double scale_factor = 3.0; // Increase overall size (adjust as needed)

    // 0 yaw = facing "north" along +Y in our top-down map
    // We'll interpret 'front' as +Y at zero yaw.
    Eigen::Vector3d front_vertex(0.0, scale_factor * markersize / 2.0, 0.0);          // Forward
    Eigen::Vector3d rear_left_vertex(-scale_factor * markersize / 3.0, -scale_factor * markersize / 2.0, 0.0); 
    Eigen::Vector3d rear_right_vertex(scale_factor * markersize / 3.0, -scale_factor * markersize / 2.0, 0.0);

    // --- Convert from NED yaw (clockwise from North) to standard Z-up yaw (counterclockwise from +Y) ---
    // A positive NED yaw rotates clockwise, but standard 2D rotation is counterclockwise in XY-plane.
    // So we flip the sign:
    double local_yaw = -yaw_ned;

    // Create rotation matrix for local_yaw
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << std::cos(local_yaw), -std::sin(local_yaw), 0.0,
                       std::sin(local_yaw),  std::cos(local_yaw), 0.0,
                       0.0,                  0.0,                 1.0;

    // Rotate vertices
    front_vertex      = rotation_matrix * front_vertex;
    rear_left_vertex  = rotation_matrix * rear_left_vertex;
    rear_right_vertex = rotation_matrix * rear_right_vertex;

    // Create the triangle mesh
    auto vehicle_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    vehicle_mesh->vertices_ = {
        front_vertex,
        rear_left_vertex,
        rear_right_vertex
    };
    vehicle_mesh->triangles_ = {{0, 1, 2}}; // single triangle

    // Uniform green color for all vertices
    vehicle_mesh->vertex_colors_ = {
        {0.0, 1.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 1.0, 0.0}
    };

    return vehicle_mesh;
}
