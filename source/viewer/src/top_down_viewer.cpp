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
    const std::vector<Eigen::Vector3f>& points,          // Points in NED frame
    const Eigen::Vector3f& vehicle_position,            // Vehicle position in NED frame
    const std::vector<Eigen::Vector3i>& grayscale_values, // Grayscale values per point
    float mapRes                                        // Resolution of the map (size of a voxel)
) {
    if (points.size() != grayscale_values.size()) {
        throw std::invalid_argument("Points and grayscale_values must have the same size.");
    }

    // Data structure to store the brightest voxel per unique XY key
    struct VoxelInfo {
        Eigen::Vector3i gridIndex; // Voxel grid index
        int grayscale;             // Grayscale value
    };

    // Function to generate a unique key for XY indices in NED frame
    auto generateKey = [](int x, int y) -> int64_t {
        return (static_cast<int64_t>(x) << 32) | (static_cast<uint32_t>(y));
    };

    // Function to merge two maps, keeping the voxel with the highest grayscale value
    auto merge_maps = [](tsl::robin_map<int64_t, VoxelInfo>& a, const tsl::robin_map<int64_t, VoxelInfo>& b) {
        for (const auto& [key, voxel_info] : b) {
            auto it = a.find(key);
            if (it == a.end() || voxel_info.grayscale > it->second.grayscale) {
                a[key] = voxel_info;
            }
        }
    };

    // Parallel reduce to find the brightest voxel per unique XY key
    tsl::robin_map<int64_t, VoxelInfo> brightest_voxels = tbb::parallel_reduce(
        tbb::blocked_range<uint64_t>(0, points.size()),
        tsl::robin_map<int64_t, VoxelInfo>(), // Initial map
        [&](const tbb::blocked_range<uint64_t>& range, tsl::robin_map<int64_t, VoxelInfo> local_map) -> tsl::robin_map<int64_t, VoxelInfo> {
            for (uint64_t i = range.begin(); i < range.end(); ++i) {
                // Translate point relative to the vehicle position in NED frame
                Eigen::Vector3f translated_point = points[i] - vehicle_position;

                // Convert translated point to grid index
                Eigen::Vector3f scaledPos = translated_point * (1.0f / mapRes);
                Eigen::Vector3i gridIndex(
                    static_cast<int>(std::floor(scaledPos.x())),  // Northing -> Grid X
                    static_cast<int>(std::floor(scaledPos.y())),  // Easting -> Grid Y
                    static_cast<int>(std::floor(scaledPos.z()))   // Down -> Grid Z
                );

                // Generate a unique key for the XY grid index
                int64_t key = generateKey(gridIndex.x(), gridIndex.y());

                // Update the local map with the brightest voxel
                int grayscale_value = grayscale_values[i].x(); // Access grayscale intensity
                auto it = local_map.find(key);
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
        // Convert grid index back to NED position
        Eigen::Vector3f voxelPos = voxel_info.gridIndex.cast<float>() * mapRes;

        // Compute square corners in the NED frame
        float half_size = mapRes / 2.0f;
        std::vector<Eigen::Vector3d> corners = {
            {voxelPos.x() - half_size, voxelPos.y() - half_size, voxelPos.z()}, // Bottom-left
            {voxelPos.x() + half_size, voxelPos.y() - half_size, voxelPos.z()}, // Bottom-right
            {voxelPos.x() + half_size, voxelPos.y() + half_size, voxelPos.z()}, // Top-right
            {voxelPos.x() - half_size, voxelPos.y() + half_size, voxelPos.z()}  // Top-left
        };

        // Assign color (normalize grayscale to [0, 1])
        Eigen::Vector3d color = Eigen::Vector3d::Constant(voxel_info.grayscale / 255.0);

        // Add vertices
        uint64_t base_index = voxel_mesh->vertices_.size();
        voxel_mesh->vertices_.insert(voxel_mesh->vertices_.end(), corners.begin(), corners.end());
        voxel_mesh->vertex_colors_.insert(voxel_mesh->vertex_colors_.end(), 4, color);

        // Add triangles for the square (two triangles per square)
        voxel_mesh->triangles_.emplace_back(base_index, base_index + 1, base_index + 2); // Bottom-right triangle
        voxel_mesh->triangles_.emplace_back(base_index, base_index + 2, base_index + 3); // Top-left triangle
    }

    return voxel_mesh;
}

// -----------------------------------------------------------------------------
// Section: CreateVehicleMesh
// -----------------------------------------------------------------------------

std::shared_ptr<open3d::geometry::TriangleMesh> TopDownViewer::CreateVehicleMesh(float markersize, double yaw_ned /* in radians, 0=North, increases clockwise */) {
    if (markersize <= 0) {
        throw std::invalid_argument("Marker size must be positive.");
    }

    // Scale factor to adjust the size of the triangle
    double scale_factor = 3.0;

    // Define the triangle vertices in the vehicle's local frame
    // "Front" vertex at (North-facing), and rear vertices symmetric around it
    Eigen::Vector3d front_vertex(markersize * scale_factor / 2.0, 0.0, 0.0);  // Front points North (X-axis in NED)
    Eigen::Vector3d rear_left_vertex(-markersize * scale_factor / 3.0, -markersize * scale_factor / 2.0, 0.0);  // Rear left
    Eigen::Vector3d rear_right_vertex(-markersize * scale_factor / 3.0, markersize * scale_factor / 2.0, 0.0);  // Rear right

    // Convert NED yaw (clockwise from North) to standard 2D rotation
    double local_yaw = -yaw_ned; // Negate because standard rotation is counterclockwise

    // Create a 2D rotation matrix for yaw
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << std::cos(local_yaw), -std::sin(local_yaw), 0.0,
                       std::sin(local_yaw),  std::cos(local_yaw),  0.0,
                       0.0,                  0.0,                  1.0;

    // Rotate the vertices to align with the given yaw in the NED frame
    front_vertex      = rotation_matrix * front_vertex;
    rear_left_vertex  = rotation_matrix * rear_left_vertex;
    rear_right_vertex = rotation_matrix * rear_right_vertex;

    // Create the triangle mesh
    auto vehicle_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    vehicle_mesh->vertices_ = {front_vertex, rear_left_vertex, rear_right_vertex};
    vehicle_mesh->triangles_ = {{0, 1, 2}}; // Single triangle (front, rear-left, rear-right)

    // Set uniform color (green) for the vehicle mesh
    vehicle_mesh->vertex_colors_ = {
        {0.0, 1.0, 0.0}, // Front vertex
        {0.0, 1.0, 0.0}, // Rear-left vertex
        {0.0, 1.0, 0.0}  // Rear-right vertex
    };

    return vehicle_mesh;
}

// -----------------------------------------------------------------------------
// Section: CreateVehicleCircleMesh
// -----------------------------------------------------------------------------

std::shared_ptr<open3d::geometry::TriangleMesh> TopDownViewer::CreateVehicleCircleMesh(float radius, int segments) {
    if (radius <= 0 || segments < 3) {
        throw std::invalid_argument("Radius must be positive and segments must be >= 3.");
    }

    auto circle_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // Center vertex of the circle
    Eigen::Vector3d center_vertex(0.0, 0.0, 0.0);
    circle_mesh->vertices_.push_back(center_vertex);

    // Generate vertices around the circle
    std::vector<Eigen::Vector3d> circle_vertices;
    for (int i = 0; i < segments; ++i) {
        double angle = 2.0 * M_PI * i / segments; // Divide the circle into equal angles
        Eigen::Vector3d vertex(radius * std::cos(angle), radius * std::sin(angle), 0.0);
        circle_vertices.push_back(vertex);
        circle_mesh->vertices_.push_back(vertex);
    }

    // Generate triangles to connect the center to the circle vertices
    for (int i = 1; i <= segments; ++i) {
        int next_index = (i % segments) + 1; // Wrap around to the first vertex
        circle_mesh->triangles_.emplace_back(0, i, next_index);
    }

    // Set a uniform color (e.g., green) for the circle
    Eigen::Vector3d color(0.0, 1.0, 0.0); // Green color
    circle_mesh->vertex_colors_.resize(circle_mesh->vertices_.size(), color);

    return circle_mesh;
}

// -----------------------------------------------------------------------------
// Section: CreateVoxelCubes
// -----------------------------------------------------------------------------

std::shared_ptr<open3d::geometry::TriangleMesh> TopDownViewer::CreateVoxelCubes(
    const std::vector<Eigen::Vector3f>& points,
    const Eigen::Vector3f& vehicle_position,
    const std::vector<Eigen::Vector3i>& grayscale_values, // Grayscale values per point
    float mapRes
) {
    if (points.size() != grayscale_values.size()) {
        throw std::invalid_argument("Points and grayscale_values must have the same size.");
    }

    if (mapRes <= 0.0f) {
        throw std::invalid_argument("Map resolution must be positive.");
    }

    // Create shared mesh
    auto cube_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // Define cube vertex offsets
    const float half_size = mapRes / 2.0f;
    const std::vector<Eigen::Vector3f> offsets = {
        {-half_size, -half_size, -half_size}, {half_size, -half_size, -half_size}, 
        {half_size,  half_size, -half_size},  {-half_size,  half_size, -half_size},
        {-half_size, -half_size,  half_size}, {half_size, -half_size,  half_size}, 
        {half_size,  half_size,  half_size},  {-half_size,  half_size,  half_size}
    };

    // Define cube faces
    const std::vector<Eigen::Vector3i> faces = {
        {0, 1, 2}, {0, 2, 3}, // Bottom face
        {4, 5, 6}, {4, 6, 7}, // Top face
        {0, 1, 5}, {0, 5, 4}, // Front face
        {2, 3, 7}, {2, 7, 6}, // Back face
        {0, 3, 7}, {0, 7, 4}, // Left face
        {1, 2, 6}, {1, 6, 5}  // Right face
    };

    // Use concurrent vectors to collect vertices, faces, and colors
    tbb::concurrent_vector<Eigen::Vector3d> vertices;
    tbb::concurrent_vector<Eigen::Vector3i> triangles;
    tbb::concurrent_vector<Eigen::Vector3d> colors;

    // Parallel loop to process all points
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i < range.end(); ++i) {
                const auto& point = points[i];
                const auto& grayscale = grayscale_values[i]; // Grayscale value as Eigen::Vector3i

                // Translate point relative to vehicle position
                Eigen::Vector3f translated_point = point - vehicle_position;

                // Convert to voxel grid center position
                Eigen::Vector3f voxel_center = (translated_point.array() / mapRes).floor() * mapRes;

                // Map grayscale to normalized color [0, 1]
                float gray = std::clamp(grayscale.x() / 255.0f, 0.0f, 1.0f); // Grayscale value as float
                Eigen::Vector3d voxel_color(gray, gray, gray); // Gray shade (R = G = B)

                // Compute cube vertices
                std::vector<Eigen::Vector3d> cube_vertices;
                for (const auto& offset : offsets) {
                    cube_vertices.push_back((voxel_center + offset).cast<double>());
                }

                // Add vertices and colors to concurrent vectors
                size_t base_index = vertices.size();
                for (const auto& vertex : cube_vertices) {
                    vertices.push_back(vertex);
                    colors.push_back(voxel_color);
                }

                // Add cube faces to the triangles vector
                for (const auto& face : faces) {
                    triangles.push_back(Eigen::Vector3i(
                        base_index + face[0], base_index + face[1], base_index + face[2]
                    ));
                }
            }
        }
    );

    // Transfer concurrent vectors into the final mesh
    cube_mesh->vertices_.assign(vertices.begin(), vertices.end());
    cube_mesh->triangles_.assign(triangles.begin(), triangles.end());
    cube_mesh->vertex_colors_.assign(colors.begin(), colors.end());

    return cube_mesh;
}

// -----------------------------------------------------------------------------
// Section: CreateVoxelCubes
// -----------------------------------------------------------------------------

std::shared_ptr<open3d::geometry::TriangleMesh> TopDownViewer::CreateVehicleSphereMesh(float radius, int latitude_segments, int longitude_segments) {
    if (radius <= 0 || latitude_segments < 2 || longitude_segments < 3) {
        throw std::invalid_argument("Radius must be positive, latitude_segments >= 2, and longitude_segments >= 3.");
    }

    auto sphere_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // Generate vertices
    for (int lat = 0; lat <= latitude_segments; ++lat) {
        double theta = M_PI * lat / latitude_segments; // Latitude angle from 0 to pi
        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);

        for (int lon = 0; lon <= longitude_segments; ++lon) {
            double phi = 2.0 * M_PI * lon / longitude_segments; // Longitude angle from 0 to 2*pi
            double x = radius * sin_theta * std::cos(phi);
            double y = radius * sin_theta * std::sin(phi);
            double z = radius * cos_theta;

            // Add vertex
            sphere_mesh->vertices_.emplace_back(x, y, z);

            // Assign a uniform color (green)
            sphere_mesh->vertex_colors_.emplace_back(Eigen::Vector3d(0.0, 1.0, 0.0)); // Green color
        }
    }

    // Generate triangles
    for (int lat = 0; lat < latitude_segments; ++lat) {
        for (int lon = 0; lon < longitude_segments; ++lon) {
            int current = lat * (longitude_segments + 1) + lon;
            int next = current + longitude_segments + 1;

            // Two triangles per quad
            sphere_mesh->triangles_.emplace_back(current, next, current + 1);
            sphere_mesh->triangles_.emplace_back(next, next + 1, current + 1);
        }
    }

    return sphere_mesh;
}


