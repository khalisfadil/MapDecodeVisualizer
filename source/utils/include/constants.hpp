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

// -----------------------------------------------------------------------------
/**
 * @brief Maximum number of points for static voxel handling.
 *
 * @detail This constant defines the upper limit on the number of points that can be stored 
 * in a **static voxel**. The value is set to 128 * 1024 * 5, indicating that each static voxel
 * can hold up to 640,000 points. This helps control memory usage and ensures consistent behavior 
 * for static voxel processing.
 */
constexpr int MAX_NUM_POINT_STATIC = 128 * 1024 * 5;  // Static voxel max points

// -----------------------------------------------------------------------------
/**
 * @brief Maximum number of points for dynamic voxel handling.
 *
 * @detail This constant defines the upper limit on the number of points that can be stored 
 * in a **dynamic voxel**. The value is set to 128 * 1024, which allows each dynamic voxel 
 * to hold up to 131,072 points. This cap helps manage memory consumption for dynamic voxel 
 * processing and ensures efficient handling of moving or changing points.
 */
constexpr int MAX_NUM_POINT_DYNAMIC = 128 * 1024;    // Dynamic voxel max points

// -----------------------------------------------------------------------------
/**
 * @brief General maximum number of points per voxel.
 *
 * @detail This constant defines the general upper limit for the number of points that can be stored 
 * in any voxel. The value is set to 128 * 1024, allowing up to 131,072 points in a voxel. It is a 
 * global cap for handling both static and dynamic voxels to ensure that voxel memory usage does not 
 * exceed a specified limit, preventing potential memory overflow or excessive computational load.
 */
constexpr int MAX_NUM_POINT = 128 * 1024;  // Maximum number of points in any voxel

// -----------------------------------------------------------------------------
/**
 * @struct MapConfig
 * @brief Configuration parameters for the map.
 *
 * @detail This structure holds the configuration settings for the map, including
 * the resolution, reaching distance, and the center point of the map. These parameters
 * define how the occupancy map is initialized and how the data is interpreted.
 */
struct MapConfig {
    float resolution = 1.0f;           ///< The resolution of the map in meters per voxel.
    float reachingDistance = 300.0f;   ///< The maximum distance that can be reached in the map.
    Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f); ///< The center point of the map in 3D space.
};

// -----------------------------------------------------------------------------
/**
 * @struct ClusterConfig
 * @brief Configuration parameters for clustering.
 *
 * @detail This structure holds the configuration settings for cluster extraction,
 * including thresholds for various clustering criteria such as size, density, velocity,
 * and similarity. These parameters are used for segmenting and analyzing data into
 * meaningful clusters.
 */
struct ClusterConfig {
    float tolerance = 0.1f;               ///< The maximum allowable distance between points in a cluster.
    int minSize = 10;                     ///< The minimum number of points in a valid cluster.
    int maxSize = 8000;                   ///< The maximum number of points allowed in a cluster.
    float staticThreshold = 0.1f;         ///< The threshold for considering static objects.
    float dynamicScoreThreshold = 0.5f;   ///< The threshold for scoring dynamic objects.
    float densityThreshold = 0.1f;        ///< The minimum density for a valid cluster.
    float velocityThreshold = 0.1f;       ///< The threshold for considering an object's velocity.
    float similarityThreshold = 0.4f;     ///< The threshold for measuring the similarity between points in a cluster.
    float maxDistanceThreshold = 10.0f;   ///< The maximum distance allowed between points in a cluster.
    float dt = 0.1f;                      ///< Time step for dynamic clustering calculations.
};