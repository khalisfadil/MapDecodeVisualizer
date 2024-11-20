#include "CallbackStaticVoxel.hpp"

CallbackStaticVoxel::CallbackStaticVoxel()
    : receivedXYZ_(MAX_NUM_POINT, Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())) 
{}

void CallbackStaticVoxel::process(const std::vector<uint8_t>& data, staticVoxel& staticVoxel) {
    if (data.empty()) return;

    if (data[0] == 0x53) { // Check header byte
        // Extract double Timestamp (8 bytes)
        double temp_t;
        std::memcpy(&temp_t, &data[1], sizeof(double)); // Bytes 1 to 8

        // Extract uint8 MaxSegment (1 byte)
        uint8_t temp_maxSegm = data[9]; // Byte 9

        // Extract uint8 SegmentIndex (1 byte)
        uint8_t temp_segm = data[10]; // Byte 10

        // Extract double North, East, Down (3 doubles, 24 bytes)
        double temp_ned[3]; // Array to hold North, East, Down
        std::memcpy(temp_ned, &data[11], 3 * sizeof(double)); // Bytes 11 to 34

        // Extract uint32 FrameID (4 bytes)
        uint32_t temp_frameID;
        std::memcpy(&temp_frameID, &data[35], sizeof(uint32_t)); // Bytes 35 to 38

        // Extract uint32 numPoints (4 bytes)
        uint32_t temp_segmNumXYZ;
        std::memcpy(&temp_segmNumXYZ, &data[39], sizeof(uint32_t)); // Bytes 39 to 42

        if (temp_frameID != frameID_) {
            if (maxNumSegment_ == numReceivedSegm_ -1) {
                // Update the subMap with points
                staticVoxel.val.resize(receivedNumXYZ_);
                std::copy(receivedXYZ_.begin(), receivedXYZ_.begin() + receivedNumXYZ_, staticVoxel.val.begin());
                staticVoxel.numVal = receivedNumXYZ_;
                staticVoxel.frameID = frameID_;
                staticVoxel.t = t_;
                staticVoxel.NED = NED_;
            }

            // Reset state for new frame
            NED_ << temp_ned[0], temp_ned[1], temp_ned[2];
            receivedNumXYZ_ = 0;
            frameID_ = temp_frameID;
            t_ = temp_t;
            maxNumSegment_ = temp_maxSegm;
            numReceivedSegm_ = 0;
        }

        // Verify packet size ~1500 = 
        if (data.size() - 43 == temp_segmNumXYZ * 12) {
            numReceivedSegm_++;
            uint32_t temp_offset = temp_segm * 115;

            Eigen::Vector3f temp_receivedXYZ;
            for (uint32_t i = 0; i < temp_segmNumXYZ; ++i){
                float point[3];
                std::memcpy(point, &data[43 + (i * 12)], sizeof(point));
                temp_receivedXYZ << point[0], point[1], point[2];
                receivedXYZ_[i+temp_offset] = temp_receivedXYZ;
            }

            receivedNumXYZ_ = temp_offset + temp_segmNumXYZ;
        }
    }
}
