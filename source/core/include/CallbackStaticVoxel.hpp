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

constexpr int MAX_NUM_POINT = 128*1024*5;

// CallbackSubMapFrame class
class CallbackStaticVoxel {
public:

    struct staticVoxel {
        std::vector<Eigen::Vector3f> val;
        uint32_t numVal = 0;
        uint32_t frameID = 0;
        double t = 0;
        Eigen::Vector3d NED;
    };  

    CallbackStaticVoxel();

    void process(const std::vector<uint8_t>& data, staticVoxel& staticVoxel);

private:

    double t_;
    std::vector<Eigen::Vector3f> receivedXYZ_;
    uint32_t receivedNumXYZ_;
    uint8_t maxNumSegment_;
    uint8_t numReceivedSegm_;
    uint32_t frameID_;
    Eigen::Vector3d NED_;

};
