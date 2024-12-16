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

// -----------------------------------------------------------------------------
// Section: Class CallbackPoints
// -----------------------------------------------------------------------------

/**
 * @class CallbackPoints
 * 
 * @brief A class for processing incoming point cloud data a
 *  It decodes binary data packets, extracts 3D points
 * and associated metadata, and stores the information in a structured format.
 */
class CallbackPoints {

    // -----------------------------------------------------------------------------
    // Section: public Class CallbackPoints
    // -----------------------------------------------------------------------------
    public:

        // -----------------------------------------------------------------------------
        /**
         * @struct Points
         * 
         * @brief Represents a container for points data.
         * Contains 3D points, frame metadata, and associated positional/orientation data.
         */
        struct Points {
            std::vector<Eigen::Vector3f> val; ///< 3D points in the voxel.
            uint32_t numVal = 0; ///< Number of valid 3D points.
            uint32_t frameID = 0; ///< Frame ID for identifying the data.
            double t = 0; ///< Timestamp of the frame.
            Eigen::Vector3d NED; ///< North-East-Down position of the frame.
            Eigen::Vector3d RPY; ///< Roll-Pitch-Yaw orientation of the frame.

            // Custom constructor
            Points()
                : val(MAX_NUM_POINT, Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())), 
                numVal(0), frameID(0), t(0.0),
                NED(Eigen::Vector3d::Zero()),
                RPY(Eigen::Vector3d::Zero()) {}
        };   

        // -----------------------------------------------------------------------------
        /**
         * @brief Constructs the CallbackDynamicVoxel object.
         * Initializes the receivedXYZ_ buffer with NaN values to mark unused slots.
         */
        CallbackPoints();

        // -----------------------------------------------------------------------------
        /**
         * @brief Processes incoming data packets and updates the staticVoxel object.
         * 
         * @param data The raw binary data packet.
         * @param voxel Reference to the staticVoxel structure to populate with processed data.
         * 
         * The method validates the data packet format, extracts metadata (timestamp, position, orientation),
         * and appends 3D points to the buffer. If a new frame is detected, it resets internal states
         * and finalizes the previous frame.
         */
        void process(const std::vector<uint8_t>& data, Points& points);

    private:

        // -----------------------------------------------------------------------------
        /**
         * @brief Current timestamp of the frame being processed.
         * 
         * @details This variable holds the timestamp of the current frame in seconds. 
         * It is used to synchronize processing and track the time of the current frame 
         * in the data stream or system timeline.
         */
        double t_;

        // -----------------------------------------------------------------------------
        /**
         * @brief Buffer for received 3D points.
         * 
         * @details A vector containing the received 3D points (x, y, z) for the current frame. 
         * Each element of the vector represents a point in 3D space in floating-point format.
         */
        std::vector<Eigen::Vector3f> receivedXYZ_;

        // -----------------------------------------------------------------------------
        /**
         * @brief Total number of received 3D points in the current frame.
         * 
         * @details This variable keeps track of the number of 3D points received in the current 
         * frame. It is used to validate data completeness and manage processing.
         */
        uint32_t receivedNumXYZ_;

        // -----------------------------------------------------------------------------
        /**
         * @brief Maximum number of segments in the current frame.
         * 
         * @details Indicates the upper limit of segments (subdivisions) in the current frame. 
         * This is used for segment-wise processing and ensures that the frame is divided 
         * into manageable parts for analysis.
         */
        uint32_t maxNumSegment_;

        // -----------------------------------------------------------------------------
        /**
         * @brief Number of segments received so far in the current frame.
         * 
         * @details This variable tracks the number of segments that have been received 
         * during the processing of the current frame. It helps in monitoring progress 
         * and ensuring all segments are accounted for.
         */
        uint32_t currSegmIdx_;

        // -----------------------------------------------------------------------------
        /**
         * @brief Frame ID of the current frame being processed.
         * 
         * @details A unique identifier for the current frame, used to distinguish 
         * between frames in the data stream and ensure proper processing order.
         */
        uint32_t frameID_;

        // -----------------------------------------------------------------------------
        /**
         * @brief North-East-Down position of the current frame.
         * 
         * @details Stores the NED position (x, y, z) of the current frame in meters. 
         * This information provides the geographic location or orientation reference 
         * for the frame.
         */
        Eigen::Vector3d NED_;

        // -----------------------------------------------------------------------------
        /**
         * @brief Roll-Pitch-Yaw orientation of the current frame.
         * 
         * @details Stores the orientation of the current frame in RPY (roll, pitch, yaw) format, 
         * measured in radians. This helps in understanding the rotational position of 
         * the frame relative to a fixed reference.
         */
        Eigen::Vector3d RPY_;

        uint32_t segmentTracker;
};