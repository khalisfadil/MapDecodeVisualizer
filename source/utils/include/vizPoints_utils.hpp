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

#include "top_down_viewer.hpp"

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
         * @brief Stores the voxel centers of the static map.
         *
         * This vector contains the 3D positions of voxel centers that represent the static map.
         * It is updated in real-time and shared across threads for visualization or further processing.
         */
        static std::vector<Eigen::Vector3f> receivedStaticVoxels;

        // -----------------------------------------------------------------------------
        /**
         * @brief Stores colors corresponding to voxel occupancy states.
         *
         * Each element represents the RGB color encoding of a voxel's occupancy state.
         * It is used for rendering the occupancy map in visualization pipelines.
         */
        static std::vector<Eigen::Vector3i> receivedOccupancyColors;

        // -----------------------------------------------------------------------------
        /**
         * @brief Stores colors corresponding to voxel reflectivity values.
         *
         * This vector contains the RGB color encoding of reflectivity values for each voxel.
         * Reflectivity values are typically derived from sensor data and used in visualization.
         */
        static std::vector<Eigen::Vector3i> receivedReflectivityColors;

        // -----------------------------------------------------------------------------
        /**
         * @brief Stores colors corresponding to voxel intensity values.
         *
         * Each element represents the RGB color encoding of intensity values for each voxel.
         * Intensity values are typically derived from sensor readings and visualized to indicate
         * the strength of the sensor return signal.
         */
        static std::vector<Eigen::Vector3i> receivedIntensityColors;

        // -----------------------------------------------------------------------------
        /**
         * @brief Stores colors corresponding to voxel Near-Infrared (NIR) values.
         *
         * This vector contains the RGB color encoding of NIR data for each voxel. NIR values
         * are often used in specialized visualization tasks or applications requiring spectral data.
         */
        static std::vector<Eigen::Vector3i> receivedNIRColors;

        // -----------------------------------------------------------------------------
        /**
         * @brief Condition variable to signal availability of new point data.
         *
         * This condition variable is used to notify waiting threads when new point cloud data
         * is available for processing.
         */
        static std::condition_variable pointsDataReadyCV;
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Condition variable to signal availability of new attribute data.
         *
         * This condition variable is used to notify waiting threads when new point attribute data
         * (e.g., intensity, reflectivity, NIR) is available for processing.
         */
        static std::condition_variable attributesDataReadyCV;
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Flag indicating whether new point cloud data is available.
         *
         * This atomic flag is set to true when new point cloud data is ready for processing,
         * and false when data has been consumed.
         */
        static std::atomic<bool> pointsDataAvailable;
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Flag indicating whether new attribute data is available.
         *
         * This atomic flag is set to true when new point attribute data (e.g., intensity, reflectivity, NIR)
         * is ready for processing, and false when data has been consumed.
         */
        static std::atomic<bool> attributesDataAvailable;

        // -----------------------------------------------------------------------------
        /**
         * @brief Stores the current frame ID.
         *
         * This variable tracks the frame ID of the latest data being processed. It is used
         * for synchronization and to ensure consistency across point cloud and attribute data.
         */
        static uint32_t frameID;
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Stores the current position of the vehicle or reference point.
         *
         * This vector represents the 3D position (X, Y, Z) of the vehicle or a reference point
         * in the global frame. It is updated in real-time as new data is received.
         */
        static Eigen::Vector3f position;
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Stores the current orientation of the vehicle.
         *
         * This vector represents the roll, pitch, and yaw (RPY) orientation of the vehicle
         * in radians. It is updated in real-time as new data is received.
         */
        static Eigen::Vector3f orientation;

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
        static void startListener(boost::asio::io_context& ioContext, 
                                    const std::string& host, 
                                    uint16_t port,
                                    uint32_t bufferSize, 
                                    const std::vector<int>& allowedCores,
                                    CallbackPoints& callbackProcessor, 
                                    CallbackPoints::Points& latestPoints,
                                    std::mutex& consoleMutex,
                                    std::mutex& dataMutex, 
                                    std::condition_variable& dataReadyCV, 
                                    std::atomic<bool>& dataAvailable,
                                    std::atomic<bool>& running);
        
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
        static void setThreadAffinity(const std::vector<int>& coreIDs,std::mutex& consoleMutex);

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
        * @brief Runs the occupancy map pipeline for real-time processing and updates of voxel data.
        *
        * This function processes incoming point cloud data and attributes in a real-time loop. It computes
        * occupancy map data, updates static voxel information, and extracts color attributes (occupancy, reflectivity,
        * intensity, and NIR) for visualization or further processing. The function operates at a target
        * frame rate of 10 Hz and utilizes mutexes for thread-safe data access.
        *
        * @param points Reference to the CallbackPoints object containing point cloud data.
        * @param attributes Reference to the CallbackPoints object containing attribute data (intensity, reflectivity, NIR).
        * @param frameID Reference to the current frame ID being processed.
        * @param position Reference to the current position of the vehicle (updated in the loop).
        * @param orientation Reference to the current orientation of the vehicle (updated in the loop).
        * @param staticVoxels Reference to the vector containing voxel centers for the static map.
        * @param occupancyColors Reference to the vector containing colors representing voxel occupancy states.
        * @param reflectivityColors Reference to the vector containing colors representing voxel reflectivity values.
        * @param intensityColors Reference to the vector containing colors representing voxel intensity values.
        * @param NIRColors Reference to the vector containing colors representing voxel NIR values.
        * @param allowedCores Vector of CPU core IDs that the processing thread is allowed to use.
        * @param running Atomic flag indicating whether the pipeline should continue running (set to false to exit).
        * @param consoleMutex Mutex for synchronizing console output.
        * @param pointsMutex Mutex for synchronizing access to point data.
        * @param attributesMutex Mutex for synchronizing access to attribute data.
        *
        * @note The function assumes that the number of points in the `points` and `attributes` objects match,
        *       and their respective frame IDs are synchronized. If the data is inconsistent, the function
        *       skips processing for the current cycle.
        *
        * @note The function targets a fixed processing rate of 10 Hz. If processing exceeds this duration,
        *       a warning is logged, and the function continues without delay.
        *
        * @return void
        *
        * @throws std::runtime_error If `occupancyMapInstance` methods encounter errors (implementation-dependent).
        */
        static void runOccupancyMapPipeline(CallbackPoints::Points& points, CallbackPoints::Points& attributes,
                                             uint32_t& frameID,
                                             Eigen::Vector3f& position,
                                             Eigen::Vector3f& orientation, 
                                             std::vector<Eigen::Vector3f>& staticVoxels, 
                                             std::vector<Eigen::Vector3i>& occupancyColors, 
                                             std::vector<Eigen::Vector3i>& reflectivityColors,
                                             std::vector<Eigen::Vector3i>& intensityColors, 
                                             std::vector<Eigen::Vector3i>& NIRColors,
                                             const std::vector<int>& allowedCores,
                                             std::atomic<bool>& running,
                                             std::mutex& consoleMutex,
                                             std::mutex& pointsMutex,
                                             std::mutex& attributesMutex);

        // -----------------------------------------------------------------------------
        /**
         * @brief Runs the occupancy map viewer for real-time visualization of voxel data and vehicle position.
         *
         * This function creates a visualization window using Open3D and updates it at a fixed rate (10 Hz)
         * with the latest occupancy map data and vehicle position. The map is rendered as a collection of 
         * voxel squares, and the vehicle is represented as a triangular mesh. The function also handles
         * user interaction, allowing the application to terminate gracefully when the visualization window
         * is closed.
         *
         * @param frameID The current frame ID of the occupancy map data (updated during visualization).
         * @param position The vehicle's current position in the global frame (updated during visualization).
         * @param orientation The vehicle's current orientation (yaw is used for the visualization).
         * @param staticVoxels A vector of voxel centers representing the static map.
         * @param occupancyColors A vector of colors corresponding to the occupancy state of each voxel.
         * @param reflectivityColors A vector of colors corresponding to the reflectivity of each voxel.
         * @param intensityColors A vector of colors corresponding to the intensity values of each voxel.
         * @param NIRColors A vector of colors corresponding to the NIR values of each voxel.
         * @param allowedCores A vector of CPU core IDs that the thread is allowed to run on.
         * @param running An atomic flag to indicate whether the viewer is running (set to false to exit).
         * @param consoleMutex A mutex for thread-safe console logging.
         * @param pointsMutex A mutex for synchronizing access to point data.
         * @param attributesMutex A mutex for synchronizing access to attribute data.
         *
         * @note The function assumes all input vectors (`staticVoxels`, `occupancyColors`, `reflectivityColors`, 
         *       `intensityColors`, and `NIRColors`) have the same size. If not, the function will log an error 
         *       and skip the current update.
         *
         * @throws std::runtime_error If the Open3D visualizer fails to initialize.
         *
         * @return void
         */
        static void runOccupancyMapViewer(uint32_t& frameID,
                                             Eigen::Vector3f& position,
                                             Eigen::Vector3f& orientation,
                                             std::vector<Eigen::Vector3f>& staticVoxels, 
                                             std::vector<Eigen::Vector3i>& occupancyColors, 
                                             std::vector<Eigen::Vector3i>& reflectivityColors,
                                             std::vector<Eigen::Vector3i>& intensityColors, 
                                             std::vector<Eigen::Vector3i>& NIRColors,
                                             const std::vector<int>& allowedCores,
                                             std::atomic<bool>& running,
                                             std::mutex& consoleMutex,
                                             std::mutex& pointsMutex,
                                             std::mutex& attributesMutex);

        // -----------------------------------------------------------------------------

        static void SetupTopDownView(open3d::visualization::Visualizer& vis, double cameraHeight);


    // -----------------------------------------------------------------------------
    // Section: private Class vizPointsUtils
    // -----------------------------------------------------------------------------
    private:
        

};