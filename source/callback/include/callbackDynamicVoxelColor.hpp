#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <cstring>
#include <Eigen/Dense>

#include "constants.hpp"

/**
 * @class CallbackStaticVoxel
 * 
 * @brief A class for processing incoming point cloud data and managing
 * static voxel frames. It decodes binary data packets, extracts 3D points
 * and associated metadata, and stores the information in a structured format.
 */
class CallbackDynamicVoxelColor {
public:

    /**
     * @struct staticVoxel
     * 
     * @brief Represents a container for static voxel data.
     * Contains 3D points, frame metadata, and associated positional/orientation data.
     */
    struct Voxel {
        std::vector<Eigen::Matrix<uint32_t, 3, 1>> val; ///< 3D points in the voxel.
        uint32_t numVal = 0; ///< Number of valid 3D points.
        uint32_t frameID = 0; ///< Frame ID for identifying the data.
        double t = 0; ///< Timestamp of the frame.
        Eigen::Vector3d NED; ///< North-East-Down position of the frame.
        Eigen::Vector3d RPY; ///< Roll-Pitch-Yaw orientation of the frame.
        
        // Custom constructor
        Voxel()
            : val(), numVal(0), frameID(0), t(0.0),
            NED(Eigen::Vector3d::Zero()),
            RPY(Eigen::Vector3d::Zero()) {}
    };  

    /**
     * @brief Constructor for the CallbackStaticVoxel class.
     * Initializes internal states and allocates space for point cloud storage.
     */
    CallbackDynamicVoxelColor();

    /**
     * @brief Processes a binary data packet to extract voxel data.
     * 
     * @param data The binary data packet containing point cloud and metadata.
     * @param staticVoxel Reference to a staticVoxel object to store the processed data.
     */
    void process(const std::vector<uint8_t>& data, Voxel& voxel);

private:

    double t_; ///< Current timestamp of the frame being processed.
    std::vector<Eigen::Matrix<uint32_t, 3, 1>> receivedXYZ_; ///< Buffer for received 3D points.
    uint32_t receivedNumXYZ_; ///< Total number of received 3D points in the current frame.
    uint8_t maxNumSegment_; ///< Maximum number of segments in the current frame.
    uint8_t numReceivedSegm_; ///< Number of segments received so far in the current frame.
    uint32_t frameID_; ///< Frame ID of the current frame being processed.
    Eigen::Vector3d NED_; ///< North-East-Down position of the current frame.
    Eigen::Vector3d RPY_; ///< Roll-Pitch-Yaw orientation of the current frame.

};
