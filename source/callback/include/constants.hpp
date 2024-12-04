#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Maximum number of points for static and dynamic voxel handling
constexpr int MAX_NUM_POINT_STATIC = 128 * 1024 * 5; // Static voxel max points
constexpr int MAX_NUM_POINT_DYNAMIC = 128 * 1024;    // Dynamic voxel max points
constexpr int MAX_NUM_POINT = 128*1024;

struct MapConfig {
    float resolution = 1.0f;
    float reachingDistance = 300.0f;
    Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
};

struct ClusterConfig {
    float tolerance = 0.1f;
    int minSize = 10;
    int maxSize = 5000;
    float staticThreshold = 0.1f;
    float dynamicScoreThreshold = 0.5f;
    float densityThreshold = 0.1f;
    float velocityThreshold = 0.1f;
    float similarityThreshold = 0.4f;
    float maxDistanceThreshold = 10.0f;
    float dt = 0.1f;
};

#endif // CONSTANTS_HPP