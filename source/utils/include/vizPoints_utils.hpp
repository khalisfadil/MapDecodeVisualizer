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
#include <optional>
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
         * @brief Global flag indicating whether the program is running.
         *
         * @details
         * This static atomic boolean is used to control the running state of the program.
         * It is shared across all threads to signal when the program should stop execution.
         * 
         * - **Initial Value**: `true` (indicating the program is running).
         * - **Modification**: 
         *   - Set to `false` by the signal handler (`signalHandler`) upon receiving termination signals
         *     (`SIGINT` or `SIGTERM`).
         *   - Threads monitor this flag to determine when to exit their loops or terminate processing.
         * 
         * @note
         * - Declared as `std::atomic<bool>` to ensure thread-safe access and modification across multiple threads.
         * - Being static, it is shared among all instances of the `vizPointsUtils` class.
         */
        static std::atomic<bool> running;

        // -----------------------------------------------------------------------------
        /**
         * @brief Constructs the `vizPointsUtils` object and initializes static instances of key components.
         *
         * @details
         * This constructor ensures that the shared static instances of `OccupancyMap` and `ClusterExtractor`
         * are initialized if they have not already been created. These instances are configured using 
         * parameters from the `mapConfig` and `clusterConfig` objects.
         *
         * - `occupancyMapInstance`:
         *   - Initializes the shared occupancy map instance with resolution, reaching distance, and center from `mapConfig`.
         * - `clusterExtractorInstance`:
         *   - Initializes the shared cluster extractor instance with parameters from `clusterConfig` such as tolerance,
         *     size thresholds, static thresholds, and various dynamic and similarity thresholds.
         *
         * This ensures that these components are instantiated once and reused across all instances of `vizPointsUtils`.
         */
        vizPointsUtils();

        // -----------------------------------------------------------------------------
        /**
         * @brief Handles termination signals (`SIGINT` and `SIGTERM`) to initiate a graceful shutdown.
         *
         * @details
         * This static method is invoked when a termination signal (`SIGINT` or `SIGTERM`) is received by the program.
         * It performs the following tasks:
         * 
         * - Sets the `vizPointsUtils::running` flag to `false`, signaling all threads to stop execution.
         * - Notifies all threads waiting on the `vizPointsUtils::queueCV` condition variable, ensuring they can exit cleanly.
         * - Logs a shutdown message to the standard output (`STDOUT`), indicating the program is stopping its listeners
         *   and processors.
         * 
         * The function is designed to be registered as a signal handler using `sigaction` or similar mechanisms.
         *
         * @param signal
         * The signal number received by the program (`SIGINT` or `SIGTERM`).
         *
         * @note
         * The function is static because signal handlers cannot be non-static methods due to their required signature.
         * Additionally, care is taken to use `write` instead of `std::cout` to ensure reentrancy and avoid undefined behavior
         * in the signal handler context.
         */
        static void signalHandler(int signal);

        

        // -----------------------------------------------------------------------------
        /**
         * @brief Starts a UDP listener for incoming point cloud data packets.
         *
         * @details
         * This function initializes a UDP listener using Boost ASIO to receive point cloud data on the specified host and port.
         * It processes the incoming data, updates shared buffers, and notifies waiting threads. The function runs in a thread
         * and handles errors and reconnections gracefully. It operates while the `vizPointsUtils::running` flag remains true.
         *
         * - Validates the provided host and port before starting the listener.
         * - Creates and configures a `UDPSocket` to handle incoming data packets.
         * - Processes received packets using `CallbackPoints::process()` and updates thread-safe shared buffers.
         * - Notifies other threads via `pointsDataReadyCV` when new data is available.
         * - Handles errors during execution and restarts the `io_context` if needed (limited to `maxErrors` restarts).
         * - Logs events (e.g., listener start, received packets, errors, and shutdown) for monitoring.
         * - Gracefully shuts down when `vizPointsUtils::running` is set to `false`.
         *
         * @param ioContext
         * Reference to a Boost ASIO `io_context` object used for asynchronous operations.
         *
         * @param host
         * The hostname or IP address on which the listener should bind.
         *
         * @param port
         * The port number on which the listener should listen for incoming packets.
         *
         * @param bufferSize
         * Size of the buffer used for receiving data packets.
         *
         * @param allowedCores
         * A vector of integers specifying the CPU cores to which the listener thread should be pinned for affinity.
         *
         * @note
         * - The function logs errors and limits error logs to a maximum count (`maxErrors`).
         * - If the host or port is invalid, the function logs an error and exits early.
         * - The function runs `ioContext.run()` in a loop to handle multiple packets until the program stops.
         */
        void startPointsListener(boost::asio::io_context& ioContext, 
                                    const std::string& host, 
                                    uint16_t port,
                                    uint32_t bufferSize, 
                                    const std::vector<int>& allowedCores);

        // -----------------------------------------------------------------------------
        /**
         * @brief Starts a UDP listener for incoming attributes data packets.
         *
         * @details
         * This function initializes a UDP listener using Boost ASIO to receive attributes data on the specified host and port.
         * It processes the incoming data, updates shared buffers, and notifies waiting threads. The function runs in a thread
         * and handles errors and reconnections gracefully. It operates while the `vizPointsUtils::running` flag remains true.
         *
         * - Validates the provided host and port before starting the listener.
         * - Creates and configures a `UDPSocket` to handle incoming data packets.
         * - Processes received packets using `CallbackPoints::process()` and updates thread-safe shared buffers for attributes.
         * - Notifies other threads via `attributesDataReadyCV` when new data is available.
         * - Handles errors during execution and restarts the `io_context` if needed (limited to `maxErrors` restarts).
         * - Logs events (e.g., listener start, received packets, errors, and shutdown) for monitoring.
         * - Gracefully shuts down when `vizPointsUtils::running` is set to `false`.
         *
         * @param ioContext
         * Reference to a Boost ASIO `io_context` object used for asynchronous operations.
         *
         * @param host
         * The hostname or IP address on which the listener should bind.
         *
         * @param port
         * The port number on which the listener should listen for incoming packets.
         *
         * @param bufferSize
         * Size of the buffer used for receiving data packets.
         *
         * @param allowedCores
         * A vector of integers specifying the CPU cores to which the listener thread should be pinned for affinity.
         *
         * @note
         * - The function logs errors and limits error logs to a maximum count (`maxErrors`).
         * - If the host or port is invalid, the function logs an error and exits early.
         * - The function runs `ioContext.run()` in a loop to handle multiple packets until the program stops.
         */
        void startAttributesListener(boost::asio::io_context& ioContext, 
                                        const std::string& host, 
                                        uint16_t port,
                                        uint32_t bufferSize, 
                                        const std::vector<int>& allowedCores);
        
        // -----------------------------------------------------------------------------
        /**
         * @brief Sets the CPU core affinity for the current thread.
         *
         * @details
         * This function restricts the execution of the current thread to a specified set of CPU cores.
         * It uses `pthread_setaffinity_np` to apply the core affinity on platforms that support POSIX threads.
         * 
         * - Validates the provided core IDs to ensure they fall within the range of available CPU cores.
         * - Adds valid core IDs to a CPU set (`cpu_set_t`) and applies the affinity to the current thread.
         * - Logs warnings for empty or invalid core IDs and errors if setting the affinity fails.
         * - Outputs the list of cores successfully assigned to the thread.
         *
         * @param coreIDs
         * A vector of integers specifying the CPU cores to which the current thread should be pinned.
         *
         * @note
         * - If `coreIDs` is empty, the function logs a warning and does not modify thread affinity.
         * - Invalid core IDs (e.g., negative values or IDs exceeding available cores) are skipped.
         * - Uses `pthread_self()` to identify the current thread and apply the affinity.
         * - Requires the program to run on a system that supports thread affinity (e.g., Linux).
         *
         * @warning
         * - Setting thread affinity can reduce scheduling flexibility and may lead to suboptimal performance if used incorrectly.
         * - Ensure that the provided core IDs align with the system's hardware configuration.
         */
        void setThreadAffinity(const std::vector<int>& coreIDs);

        // -----------------------------------------------------------------------------
        /**
         * @brief Processes synchronized point cloud and attributes data to update the occupancy map at 10 Hz.
         *
         * @details
         * This function processes incoming point cloud and attributes data, synchronizes them by frame ID,
         * and updates the occupancy map in real-time. The pipeline runs at a target frequency of 10 Hz,
         * performing the following tasks:
         * 
         * - **Thread Affinity**:
         *   - Sets thread affinity for optimal performance on specified CPU cores.
         * 
         * - **Data Synchronization**:
         *   - Waits for new points and attributes data using condition variables.
         *   - Adds the data to deques (`pointsDeque` and `attributesDeque`) to maintain a history of the last three frames.
         *   - Searches the deques for the latest matching frame ID to ensure synchronization between points and attributes.
         * 
         * - **Occupancy Map Update**:
         *   - Extracts data from the matched frame, including points and their attributes (intensity, reflectivity, and NIR).
         *   - Runs the `occupancyMapInstance` pipeline to update the map with new point cloud data.
         *   - Processes static voxels to compute their centers and associated color attributes.
         *   - Updates shared buffers (`OMD_writeBuffer`) with the processed data for visualization.
         * 
         * - **Timing and Performance**:
         *   - Ensures the processing loop maintains a target cycle time of 100 ms (10 Hz).
         *   - Logs warnings if processing exceeds the target time.
         *
         * @param allowedCores
         * A vector of integers specifying the CPU cores to which the pipeline thread should be pinned for affinity.
         *
         * @note
         * - The function continuously runs while the static `vizPointsUtils::running` flag is `true`.
         * - Data synchronization ensures that only matching frames are processed to avoid inconsistencies.
         * - Thread-safe operations are implemented using mutexes and condition variables.
         *
         * @warning
         * - If points and attributes are not synchronized, the function logs a warning and skips processing for that cycle.
         */
        void runOccupancyMapPipeline(const std::vector<int>& allowedCores);

        // -----------------------------------------------------------------------------
        /**
         * @brief Visualizes the occupancy map and vehicle state in real time using Open3D.
         *
         * @details
         * This function runs a real-time visualization loop at a target frequency of 5 Hz (200 ms per cycle). 
         * It renders the occupancy map and vehicle orientation using Open3D's visualizer. The process includes:
         *
         * - **Thread Affinity**:
         *   - Sets the thread's CPU affinity to optimize performance on specified cores.
         *
         * - **Data Synchronization**:
         *   - Waits for the latest processed occupancy map data using a condition variable.
         *   - Copies the data from shared buffers (`OMD_readBuffer`) for rendering.
         *
         * - **Rendering**:
         *   - Clears existing geometries in the Open3D visualizer.
         *   - Creates voxel squares (static objects) and a vehicle mesh for visualization.
         *   - Adds the created geometries to the visualizer and updates the render.
         *
         * - **Error Handling**:
         *   - Exits the loop and logs an error if adding geometries fails.
         *   - Terminates gracefully if the user closes the visualizer window.
         *
         * - **Timing and Performance**:
         *   - Maintains a target cycle duration of 200 ms (5 Hz).
         *   - Logs warnings if rendering exceeds the target cycle time.
         *
         * @param allowedCores
         * A vector of integers specifying the CPU cores to which the viewer thread should be pinned for affinity.
         *
         * @note
         * - The function continuously runs while the static `vizPointsUtils::running` flag is `true`.
         * - Uses thread-safe mechanisms (`occupancyMapDataMutex` and `occupancyMapDataReadyCV`) for data access.
         * - Open3D visualizer must be correctly initialized before using this function.
         *
         * @warning
         * - Ensure that the system supports Open3D visualization and required configurations (e.g., display settings).
         * - If geometries fail to render, the visualization loop will terminate.
         */
        void runOccupancyMapViewer(const std::vector<int>& allowedCores);

        // -----------------------------------------------------------------------------
        /**
         * @brief Configures the Open3D visualizer to display a top-down view of the scene.
         *
         * @details
         * This function sets up the Open3D visualizer to use a top-down camera perspective with the following configurations:
         *
         * - **Background Color**:
         *   - Sets the background color of the visualizer to black (`RGB: 0.0, 0.0, 0.0`).
         *
         * - **Camera Extrinsics**:
         *   - Positions the camera directly above the origin at a height specified by `cameraHeight`.
         *   - Ensures the camera is looking straight down towards the origin.
         *
         * - **Zoom Level**:
         *   - Sets the zoom level of the visualizer to provide an optimal field of view.
         *
         * - **Render Refresh**:
         *   - Updates the visualizer to apply the new camera settings and refresh the view.
         *
         * Debugging information, such as the applied camera extrinsics and zoom level, is logged to the console for verification.
         *
         * @param vis
         * Reference to an Open3D `Visualizer` object that is configured for the top-down view.
         *
         * @param cameraHeight
         * The height at which the camera is positioned above the origin (positive value for proper setup).
         *
         * @note
         * - The camera extrinsics are set to ensure the camera is aligned directly along the Z-axis.
         * - The zoom level may need adjustment depending on the size and scale of the scene.
         *
         * @warning
         * - Ensure that the `vis` object is properly initialized before calling this function.
         * - Camera settings may overwrite existing visualizer configurations.
         */
        void SetupTopDownView(open3d::visualization::Visualizer& vis, double cameraHeight);

    // -----------------------------------------------------------------------------
    // Section: private Class vizPointsUtils
    // -----------------------------------------------------------------------------
    private:

        // -----------------------------------------------------------------------------
        /**
         * @brief Stores processed occupancy map data for visualization.
         *
         * @details
         * This structure holds the information required to render the occupancy map, including:
         * - `frameID`: The ID of the processed frame.
         * - `position`: The position of the vehicle in the NED coordinate system.
         * - `orientation`: The orientation of the vehicle as roll, pitch, and yaw.
         * - `staticVoxels`: A vector of voxel center positions in the map.
         * - `occupancyColors`: Colors associated with voxel occupancy status.
         * - `reflectivityColors`: Colors representing reflectivity data.
         * - `intensityColors`: Colors representing intensity data.
         * - `NIRColors`: Colors representing near-infrared data.
         */
        struct OccupancyMapData {
            uint32_t frameID;
            Eigen::Vector3f position;
            Eigen::Vector3f orientation;
            std::vector<Eigen::Vector3f> staticVoxels;
            std::vector<Eigen::Vector3i> occupancyColors;
            std::vector<Eigen::Vector3i> reflectivityColors;
            std::vector<Eigen::Vector3i> intensityColors;
            std::vector<Eigen::Vector3i> NIRColors;
        };

        // -----------------------------------------------------------------------------
        /**
         * @brief Double-buffered storage for occupancy map data.
         *
         * @details
         * These buffers are used to store processed occupancy map data. One buffer (`OMD_writeBuffer`) 
         * is used for writing new data, while the other (`OMD_readBuffer`) is used for reading.
         */
        OccupancyMapData OMD_buffer1, OMD_buffer2;

        // -----------------------------------------------------------------------------
        /**
         * @brief Pointers to the active write and read buffers for occupancy map data.
         *
         * @details
         * - `OMD_writeBuffer`: Points to the buffer used for writing new occupancy map data.
         * - `OMD_readBuffer`: Points to the buffer used for reading processed occupancy map data.
         */
        OccupancyMapData* OMD_writeBuffer = &OMD_buffer1;

        // -----------------------------------------------------------------------------
        /**
         * @brief Double-buffered storage for points data.
         *
         * @details
         * These buffers are used to store incoming points data. One buffer is used for writing new 
         * points data (`P_writeBuffer`), while the other is used for reading processed points data (`P_readBuffer`).
         */
        OccupancyMapData* OMD_readBuffer = &OMD_buffer2;

        // -----------------------------------------------------------------------------
        /**
         * @brief Pointers to the active write and read buffers for points data.
         *
         * @details
         * - `P_writeBuffer`: Points to the buffer used for writing new points data.
         * - `P_readBuffer`: Points to the buffer used for reading processed points data.
         */
        CallbackPoints::Points P_buffer1, P_buffer2;

        // -----------------------------------------------------------------------------
        /**
         * @brief Double-buffered storage for attributes data.
         *
         * @details
         * These buffers are used to store incoming attributes data. One buffer is used for writing 
         * new attributes data (`A_writeBuffer`), while the other is used for reading processed attributes data (`A_readBuffer`).
         */
        CallbackPoints::Points* P_writeBuffer = &P_buffer1;

        // -----------------------------------------------------------------------------
                /**
         * @brief Double-buffered storage for attributes data.
         *
         * @details
         * These buffers are used to store incoming attributes data. One buffer is used for writing 
         * new attributes data (`A_writeBuffer`), while the other is used for reading processed attributes data (`A_readBuffer`).
         */
        CallbackPoints::Points* P_readBuffer = &P_buffer2;

        // -----------------------------------------------------------------------------
                /**
         * @brief Double-buffered storage for attributes data.
         *
         * @details
         * These buffers are used to store incoming attributes data. One buffer is used for writing 
         * new attributes data (`A_writeBuffer`), while the other is used for reading processed attributes data (`A_readBuffer`).
         */
        CallbackPoints::Points A_buffer1, A_buffer2;

        // -----------------------------------------------------------------------------
        /**
         * @brief Pointers to the active write and read buffers for attributes data.
         *
         * @details
         * - `A_writeBuffer`: Points to the buffer used for writing new attributes data.
         * - `A_readBuffer`: Points to the buffer used for reading processed attributes data.
         */
        CallbackPoints::Points* A_writeBuffer = &A_buffer1;

        // -----------------------------------------------------------------------------
        /**
         * @brief Pointers to the active write and read buffers for attributes data.
         *
         * @details
         * - `A_writeBuffer`: Points to the buffer used for writing new attributes data.
         * - `A_readBuffer`: Points to the buffer used for reading processed attributes data.
         */
        CallbackPoints::Points* A_readBuffer = &A_buffer2;

        // -----------------------------------------------------------------------------
        /**
         * @brief Condition variable to signal when new occupancy map data is ready.
         *
         * @details
         * Used by threads to wait for or notify about the availability of new occupancy map data.
         */
        std::condition_variable occupancyMapDataReadyCV;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for synchronizing access to occupancy map data.
         *
         * @details
         * Ensures thread-safe access to the occupancy map buffers (`OMD_writeBuffer` and `OMD_readBuffer`).
         */
        std::mutex occupancyMapDataMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Flag indicating whether new occupancy map data is available.
         *
         * @details
         * Set to `true` when new occupancy map data is written to the buffer. Reset to `false` after reading.
         */
        std::atomic<bool> occupancyMapDataReady = false;

        // -----------------------------------------------------------------------------
        /**
         * @brief Condition variable to signal when new points data is ready.
         *
         * @details
         * Used by threads to wait for or notify about the availability of new points data.
         */
        std::condition_variable pointsDataReadyCV;

        // -----------------------------------------------------------------------------
        /**
         * @brief Flag indicating whether new points data is available.
         *
         * @details
         * Set to `true` when new points data is written to the buffer. Reset to `false` after reading.
         */
        std::atomic<bool> pointsDataReady = false;;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for synchronizing access to points data.
         *
         * @details
         * Ensures thread-safe access to the points buffers (`P_writeBuffer` and `P_readBuffer`).
         */
        std::mutex vizPointsUtils::pointsMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Condition variable to signal when new attributes data is ready.
         *
         * @details
         * Used by threads to wait for or notify about the availability of new attributes data.
         */
        std::condition_variable attributesDataReadyCV;

        // -----------------------------------------------------------------------------
        /**
         * @brief Flag indicating whether new attributes data is available.
         *
         * @details
         * Set to `true` when new attributes data is written to the buffer. Reset to `false` after reading.
         */
        std::atomic<bool> attributesDataReady;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for synchronizing access to attributes data.
         *
         * @details
         * Ensures thread-safe access to the attributes buffers (`A_writeBuffer` and `A_readBuffer`).
         */
        std::mutex vizPointsUtils::attributesMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Static condition variable for signaling across multiple threads.
         *
         * @details
         * Used globally by threads for coordination and notification during shutdown or synchronization tasks.
         */
        static std::condition_variable queueCV;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for synchronizing console output.
         *
         * @details
         * Ensures thread-safe logging and prevents interleaved console output from multiple threads.
         */
        std::mutex consoleMutex;

        // -----------------------------------------------------------------------------
        /**
         * @brief Configuration parameters for cluster extraction.
         *
         * @details
         * Stores various thresholds and settings used by the `ClusterExtractor` for analyzing clusters in point cloud data.
         */
        ClusterConfig clusterConfig;

        // -----------------------------------------------------------------------------
        /**
         * @brief Configuration parameters for the occupancy map.
         *
         * @details
         * Stores parameters such as resolution, center position, and reaching distance for initializing and updating the occupancy map.
         */
        MapConfig mapConfig;

        // -----------------------------------------------------------------------------
        /**
         * @brief Deque to store a history of recent points data.
         *
         * @details
         * Used for synchronizing points and attributes data by maintaining a sliding window of the last three frames.
         */
        std::deque<CallbackPoints::Points> pointsDeque;

        // -----------------------------------------------------------------------------
        /**
         * @brief Deque to store a history of recent attributes data.
         *
         * @details
         * Used for synchronizing points and attributes data by maintaining a sliding window of the last three frames.
         */
        std::deque<CallbackPoints::Points> attributesDeque;

        // -----------------------------------------------------------------------------
        /**
         * @brief Mutex for synchronizing access to `pointsDeque` and `attributesDeque`.
         *
         * @details
         * Ensures thread-safe operations when adding or searching for synchronized data in the deques.
         */

        std::mutex dequeMutex;

};