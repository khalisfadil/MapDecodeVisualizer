#include "callbackPoints.hpp"

// -----------------------------------------------------------------------------
// Section: CallbackPoints
// -----------------------------------------------------------------------------

CallbackPoints::CallbackPoints()
    : receivedXYZ_(MAX_NUM_POINT, Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())) 
{}

// -----------------------------------------------------------------------------
// Section: process
// -----------------------------------------------------------------------------

void CallbackPoints::process(const std::vector<uint8_t>& data, Points& points) {
    // Check if the input data is valid
    if (data.empty()) {
        std::cout << "[0] data.empty()"<< std::endl;
        return;
    }
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
        uint32_t temp_numXYZ;
        std::memcpy(&temp_numXYZ, &data[69], sizeof(uint32_t)); // Bytes 69 to 72

        std::cout << "[a] temp_maxSegm: "  << temp_maxSegm << std::endl;
        std::cout << "[a] temp_segm: "  << temp_segm << std::endl;

        std::cout << "[c] temp_frameID: "  << temp_frameID << std::endl;
        std::cout << "[c] frameID_: "  << frameID_ << std::endl;

        // Handle a new frame
        if (temp_frameID != frameID_) {
            std::cout << "[1] invoke 'temp_frameID != frameID_'"<< std::endl;
            
            std::cout << "[b] maxNumSegment_: "  << maxNumSegment_ << std::endl;
            std::cout << "[b] currSegmIdx_-1: "  << currSegmIdx_-1 << std::endl;
            std::cout << "[b] currSegmIdx_: "  << currSegmIdx_ << std::endl;
            // Finalize the previous frame if all segments are received
            if (maxNumSegment_ == currSegmIdx_-1) {
                std::cout << "[2] invoke 'maxNumSegment_ == currSegmIdx_-1'"<< std::endl;
                std::copy(receivedXYZ_.begin(), receivedXYZ_.begin() + receivedNumXYZ_, points.val.begin());
                points.numVal = receivedNumXYZ_;
                points.frameID = frameID_;
                points.t = t_;
                points.NED = NED_;
                points.RPY = RPY_;
                
            }
            // Reset internal states for the new frame
            NED_ << temp_ned[0], temp_ned[1], temp_ned[2];
            RPY_ << temp_rpy[0], temp_rpy[1], temp_rpy[2];
            receivedNumXYZ_ = 0;
            frameID_ = temp_frameID;
            t_ = temp_t;
            maxNumSegment_ = temp_maxSegm;
            currSegmIdx_ = 0;
            std::cout << "[Debug] Resetting currSegmIdx_ to 0 for new frame" << maxNumSegment_ << std::endl;

        }

        // Validate packet size and process 3D points
        if (data.size() - 73 == temp_numXYZ * 12) {
            std::cout << "[3] data.size() - 73 == temp_numXYZ * 12'"<< std::endl;
            std::cout << "[d] pointsize: "  << data.size()- 73 << std::endl;
            std::cout << "[d] temp_numSize: "  << temp_numXYZ * 12 << std::endl;
            currSegmIdx_++;
            std::cout << "[Debug] Increment current currSegmIdx_"<< maxNumSegment_ << std::endl;
            std::cout << "[Debug] Before current currSegmIdx_"<< maxNumSegment_ - 1 << std::endl;

            if (currSegmIdx_ > maxNumSegment_) {
                std::cerr << "[Error] currSegmIdx_ exceeds maxNumSegment_: " << currSegmIdx_ << std::endl;
            }

            uint32_t temp_offset = temp_segm * 110;

            Eigen::Vector3f temp_receivedXYZ;
            for (uint32_t i = 0; i < temp_numXYZ; ++i) {
                float point[3];
                std::memcpy(point, &data[73 + (i * 12)], sizeof(point));
                temp_receivedXYZ << point[0], point[1], point[2];
                receivedXYZ_[i + temp_offset] = temp_receivedXYZ;
            }

            receivedNumXYZ_ = temp_offset + temp_numXYZ;
            
        }
    }
}
