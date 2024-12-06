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

#include "udpSocket.hpp"

#include "occupancyMap.hpp"
#include "clusterExtractor.hpp"

#include "callbackPoints.hpp"
#include "vizPoints_utils.hpp"

#include <boost/asio.hpp>
#include <mutex>
#include <thread>
#include <iostream>
#include <vector>
#include <csignal>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <chrono>
#include <sched.h>
#include <pthread.h>

// -----------------------------------------------------------------------------
// Section: Class vizPointsUtils
// -----------------------------------------------------------------------------

/**
 * @class vizPointsUtils
 * @brief A utility class for managing a 3D occupancy map and performing cluster extraction.
 *
 * @detail The `vizPointsUtils` class provides functionality for setting up and managing
 * the core components of a 3D voxel-based system, including an occupancy map and cluster extraction.
 * It contains static methods and members to initialize shared resources, handle thread management, 
 * and process point cloud data. The class utilizes static configuration structures (`MapConfig`, `ClusterConfig`) 
 * for map resolution, cluster thresholds, and other parameters, ensuring consistent behavior throughout the program.
 * The class supports asynchronous UDP listeners, point and attribute processing, and various utility functions 
 * related to spatial mapping and clustering.
 */
class vizPointsUtils {

    // -----------------------------------------------------------------------------
    // Section: public Class vizPointsUtils
    // -----------------------------------------------------------------------------
    public:

        // -----------------------------------------------------------------------------
        /**
         * @brief Shared instance of the OccupancyMap object.
         *
         * @detail This static unique pointer holds a shared instance of the `OccupancyMap` class, 
         * ensuring that only one instance is used across all instances of the class.
         */
        static std::unique_ptr<OccupancyMap> occupancyMapInstance;

        // -----------------------------------------------------------------------------
        /**
         * @brief Shared instance of the ClusterExtractor object.
         *
         * @detail This static unique pointer holds a shared instance of the `ClusterExtractor` class, 
         * which is used for cluster analysis. It is used across all instances of the class for efficient 
         * and consistent cluster extraction.
         */
        static std::unique_ptr<ClusterExtractor> clusterExtractorInstance;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for synchronizing console output.
         *
         * @detail This mutex ensures that console output is accessed and modified in a thread-safe 
         * manner, preventing interleaving of log messages when multiple threads are involved.
         */
        static std::mutex consoleMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Atomic flag indicating whether the program is running.
         *
         * @detail This flag is used to control the termination of the program. It is set to `false` 
         * when the program needs to stop, and its atomic nature ensures safe access across multiple threads.
         */
        static std::atomic<bool> running;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for protecting access to points data.
         *
         * @detail This mutex is used to ensure thread-safe access and modification of points data, 
         * preventing race conditions when multiple threads are interacting with the points object.
         */
        static std::mutex pointsMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for protecting access to attributes data.
         *
         * @detail This mutex ensures that any access to attributes data is synchronized across threads 
         * to avoid conflicts and ensure consistent updates to the attributes object.
         */
        static std::mutex attributesMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for protecting access to the occupancy map.
         *
         * @detail This mutex is used to protect access to the occupancy map, ensuring thread-safety 
         * when manipulating the map data structure.
         */
        static std::mutex occupancyMapMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Condition variable for signaling between threads.
         *
         * @detail This condition variable is used to notify threads waiting for data processing to proceed, 
         * providing a mechanism for synchronization and coordination between threads.
         */
        static std::condition_variable queueCV;

        // -----------------------------------------------------------------------------
        /**
         * @brief Initializes shared resources if they have not been initialized.
         *
         * @detail This function checks whether the static instances of `occupancyMapInstance` and
         * `clusterExtractorInstance` have been initialized. If they have not, it initializes them 
         * using configuration parameters defined in `MapConfig` and `ClusterConfig` respectively.
         * This ensures that the shared resources (occupancy map and cluster extractor) are set up 
         * only once and are ready for use in the program.
         */
        static void initialize();

        // -----------------------------------------------------------------------------
        /**
         * @brief Starts a UDP listener for processing incoming packets.
         *
         * @detail This function initializes a UDP listener using Boost Asio to receive and process 
         * incoming packets on the specified host and port. It ensures robust operation by managing 
         * thread affinity, error handling, and logging. The received data is processed using a 
         * callback processor, and the results are stored in a shared points data structure. 
         * A condition variable and atomic flag are used to signal the availability of new data 
         * for further processing in a multithreaded environment.
         *
         * @param ioContext Boost Asio IO context for asynchronous operations.
         * @param host Hostname or IP address to bind the listener.
         * @param port Port number for the listener.
         * @param bufferSize Size of the UDP buffer.
         * @param allowedCores Cores for setting thread affinity.
         * @param callbackProcessor Handles incoming data packets.
         * @param latestPoints Reference to shared points data structure.
         * @param dataMutex Mutex for thread-safe data access.
         * @param dataReadyCV Condition variable for notifying data updates.
         * @param dataAvailable Atomic flag to indicate data readiness.
         */
        static void startListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port,
                                    uint32_t bufferSize, const std::vector<int>& allowedCores,
                                    CallbackPoints& callbackProcessor, CallbackPoints::Points& latestPoints,
                                    std::mutex& dataMutex, std::condition_variable& dataReadyCV, 
                                    std::atomic<bool>& dataAvailable);
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Sets thread affinity to specific CPU cores.
         *
         * @detail Restricts the current thread to run on the specified CPU cores provided in 
         * the coreIDs vector. Ensures that only valid core IDs within the range of available 
         * hardware cores are used. Logs errors and warnings for invalid inputs or failures 
         * during the operation. Uses `pthread_setaffinity_np` for applying the CPU affinity.
         *
         * @param coreIDs A vector of integers specifying the core IDs for thread affinity.
         */
        static void setThreadAffinity(const std::vector<int>& coreIDs);

        // -----------------------------------------------------------------------------
        /**
         * @brief Handles termination signals to gracefully shut down the application.
         *
         * @detail Responds to SIGINT and SIGTERM signals by setting the `running` flag to false, 
         * notifying all waiting threads via a condition variable, and logging a shutdown message 
         * to the standard output. Ensures the application can cleanly exit and release resources 
         * upon receiving termination signals.
         *
         * @param signal The signal number received by the application.
         */
        static void signalHandler(int signal);

        // -----------------------------------------------------------------------------
        /**
         * @brief Processes points and attributes, with thread affinity and cycle timing.
         *
         * @detail This function runs a processing loop that operates at a target frequency of 10 Hz 
         * (100 ms cycle). It checks the consistency between points and attributes data and performs 
         * processing by extracting point cloud data and attributes (intensity, reflectivity, NIR). 
         * The data is passed to an occupancy map pipeline. The function also ensures the thread is 
         * restricted to specified CPU cores and logs cycle timing performance. It adjusts the sleep 
         * duration to maintain the target cycle time and provides warnings if processing exceeds the time limit.
         *
         * @param points The points data structure containing the 3D points and associated metadata.
         * @param attributes The attributes data structure containing additional information (e.g., intensity, reflectivity).
         * @param allowedCores A vector of allowed CPU core IDs for thread affinity.
         */
        static void pointToWorkWith(CallbackPoints::Points& points, CallbackPoints::Points& attributes,
                                    const std::vector<int>& allowedCores);


    // -----------------------------------------------------------------------------
    // Section: private Class vizPointsUtils
    // -----------------------------------------------------------------------------
    private:

};