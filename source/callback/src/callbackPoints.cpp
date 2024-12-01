#include "callbackPoints.hpp"

/**
 * @brief Constructs the CallbackDynamicVoxel object.
 * Initializes the receivedXYZ_ buffer with NaN values to mark unused slots.
 */
CallbackPoints::CallbackPoints()
    : receivedXYZ_(MAX_NUM_POINT, Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())) 
{}

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
void CallbackPoints::process(const std::vector<uint8_t>& data, Voxel& voxel) {
    // Check if the input data is valid
    if (data.empty()) return;

    // Check the header byte to ensure data packet integrity
    if (data[0] == 0x53) { // Byte 0: Header byte
        
        // Extract double Timestamp (8 bytes)
        double temp_t;
        std::memcpy(&temp_t, &data[1], sizeof(double)); // Bytes 1 to 8

        // Extract uint8 MaxSegment (4 byte)
        uint32_t temp_maxSegm; 
        std::memcpy(&temp_maxSegm, &data[9], sizeof(uint32_t)); // Bytes 9 to 12

        // Extract uint8 SegmentIndex (4 byte)
        uint32_t temp_segm; 
        std::memcpy(&temp_segm, &data[13], sizeof(uint32_t)); // Bytes 13 to 16

        // Extract position and orientation (North, East, Down, Roll, Pitch, Yaw)
        double temp_ned[3];
        std::memcpy(temp_ned, &data[17], 3 * sizeof(double)); // Bytes 17 to 40
        double temp_rpy[3];
        std::memcpy(temp_rpy, &data[41], 3 * sizeof(double)); // Bytes 41 to 64

        // Extract frame metadata
        uint32_t temp_frameID;
        std::memcpy(&temp_frameID, &data[65], sizeof(uint32_t)); // Bytes 65 to 68
        uint32_t temp_segmNumXYZ;
        std::memcpy(&temp_segmNumXYZ, &data[69], sizeof(uint32_t)); // Bytes 69 to 72

        // Handle a new frame
        if (temp_frameID != frameID_) {
            // Finalize the previous frame if all segments are received
            if (maxNumSegment_ == numReceivedSegm_ - 1) {
                voxel.val.resize(receivedNumXYZ_);
                std::copy(receivedXYZ_.begin(), receivedXYZ_.begin() + receivedNumXYZ_, voxel.val.begin());
                voxel.numVal = receivedNumXYZ_;
                voxel.frameID = frameID_;
                voxel.t = t_;
                voxel.NED = NED_;
                voxel.RPY = RPY_;
            }

            // Reset internal states for the new frame
            NED_ << temp_ned[0], temp_ned[1], temp_ned[2];
            RPY_ << temp_rpy[0], temp_rpy[1], temp_rpy[2];
            receivedNumXYZ_ = 0;
            frameID_ = temp_frameID;
            t_ = temp_t;
            maxNumSegment_ = temp_maxSegm;
            numReceivedSegm_ = 0;
        }

        // Validate packet size and process 3D points
        if (data.size() - 73 == temp_segmNumXYZ * 12) {
            numReceivedSegm_++;
            uint32_t temp_offset = temp_segm * 110;

            Eigen::Vector3f temp_receivedXYZ;
            for (uint32_t i = 0; i < temp_segmNumXYZ; ++i) {
                float point[3];
                std::memcpy(point, &data[73 + (i * 12)], sizeof(point));
                temp_receivedXYZ << point[0], point[1], point[2];
                receivedXYZ_[i + temp_offset] = temp_receivedXYZ;
            }

            receivedNumXYZ_ = temp_offset + temp_segmNumXYZ;
        }
    }
}
