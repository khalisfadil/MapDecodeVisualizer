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
#include "vizPoints_utils.hpp"

// -----------------------------------------------------------------------------
// Section: declaration
// -----------------------------------------------------------------------------

/**
 * @brief Mutex for synchronizing console output operations.
 * 
 * @details Ensures thread-safe access to shared console output to avoid race 
 * conditions or interleaving of log messages from multiple threads.
 */
std::mutex vizPointsUtils::consoleMutex;

// -----------------------------------------------------------------------------
/**
 * @brief Mutex for synchronizing access to 3D point data.
 * 
 * @details Protects the shared buffer of 3D points to ensure thread-safe 
 * read/write operations across multiple threads during processing.
 */
std::mutex vizPointsUtils::pointsMutex;

// -----------------------------------------------------------------------------
/**
 * @brief Mutex for synchronizing access to attribute data.
 * 
 * @details Ensures thread-safe access to shared attribute data, preventing 
 * data races during concurrent updates and retrieval.
 */
std::mutex vizPointsUtils::attributesMutex;

// -----------------------------------------------------------------------------
/**
 * @brief Static atomic flag indicating whether the program is running.
 *
 * @detail This static flag is used to control the program's termination state. 
 * It is shared across all instances of the class and is accessed and modified 
 * to manage whether the program should continue running or stop. The flag is atomic, 
 * ensuring safe access in a multithreaded environment.
 */
std::atomic<bool> vizPointsUtils::running(true);  // Initialize as true

// -----------------------------------------------------------------------------
/**
 * @brief Condition variable for managing the processing queue.
 * 
 * @details Used to signal and synchronize threads waiting on events related 
 * to the processing queue, such as new data availability.
 */
std::condition_variable vizPointsUtils::queueCV;

// -----------------------------------------------------------------------------
/**
 * @brief Shared instance of the OccupancyMap object.
 * 
 * @details A static unique pointer that holds a shared instance of the 
 * `OccupancyMap` class. Ensures there is a single shared instance across 
 * the application for managing occupancy map data.
 */
std::unique_ptr<OccupancyMap> vizPointsUtils::occupancyMapInstance = nullptr;

// -----------------------------------------------------------------------------
/**
 * @brief Shared instance of the ClusterExtractor object.
 * 
 * @details A static unique pointer that holds a shared instance of the 
 * `ClusterExtractor` class. Provides consistent and efficient access to 
 * clustering functionality across the application.
 */
std::unique_ptr<ClusterExtractor> vizPointsUtils::clusterExtractorInstance = nullptr;

// -----------------------------------------------------------------------------
/**
 * @brief Stores the voxel centers of the static map.
 *
 * This vector contains the 3D positions of voxel centers that represent the static map.
 * It is updated in real-time and shared across threads for visualization or further processing.
 */
std::vector<Eigen::Vector3f> vizPointsUtils::receivedStaticVoxels;

// -----------------------------------------------------------------------------
/**
 * @brief Stores colors corresponding to voxel occupancy states.
 *
 * Each element represents the RGB color encoding of a voxel's occupancy state.
 * It is used for rendering the occupancy map in visualization pipelines.
 */
std::vector<Eigen::Vector3i> vizPointsUtils::receivedOccupancyColors;

// -----------------------------------------------------------------------------
/**
 * @brief Stores colors corresponding to voxel reflectivity values.
 *
 * This vector contains the RGB color encoding of reflectivity values for each voxel.
 * Reflectivity values are typically derived from sensor data and used in visualization.
 */
std::vector<Eigen::Vector3i> vizPointsUtils::receivedReflectivityColors;

// -----------------------------------------------------------------------------
/**
 * @brief Stores colors corresponding to voxel intensity values.
 *
 * Each element represents the RGB color encoding of intensity values for each voxel.
 * Intensity values are typically derived from sensor readings and visualized to indicate
 * the strength of the sensor return signal.
 */
std::vector<Eigen::Vector3i> vizPointsUtils::receivedIntensityColors;

// -----------------------------------------------------------------------------
/**
 * @brief Stores colors corresponding to voxel Near-Infrared (NIR) values.
 *
 * This vector contains the RGB color encoding of NIR data for each voxel. NIR values
 * are often used in specialized visualization tasks or applications requiring spectral data.
 */
std::vector<Eigen::Vector3i> vizPointsUtils::receivedNIRColors;

// -----------------------------------------------------------------------------
/**
 * @brief Condition variable to signal availability of new point data.
 *
 * This condition variable is used to notify waiting threads when new point cloud data
 * is available for processing.
 */
std::condition_variable vizPointsUtils::pointsDataReadyCV;

// -----------------------------------------------------------------------------
/**
 * @brief Condition variable to signal availability of new attribute data.
 *
 * This condition variable is used to notify waiting threads when new point attribute data
 * (e.g., intensity, reflectivity, NIR) is available for processing.
 */
std::condition_variable vizPointsUtils::attributesDataReadyCV;

// -----------------------------------------------------------------------------
/**
 * @brief Flag indicating whether new point cloud data is available.
 *
 * This atomic flag is set to true when new point cloud data is ready for processing,
 * and false when data has been consumed.
 */
std::atomic<bool> vizPointsUtils::pointsDataAvailable(false);

// -----------------------------------------------------------------------------
/**
 * @brief Flag indicating whether new attribute data is available.
 *
 * This atomic flag is set to true when new point attribute data (e.g., intensity, reflectivity, NIR)
 * is ready for processing, and false when data has been consumed.
 */
std::atomic<bool> vizPointsUtils::attributesDataAvailable(false);

// -----------------------------------------------------------------------------
/**
 * @brief Stores the current frame ID.
 *
 * This variable tracks the frame ID of the latest data being processed. It is used
 * for synchronization and to ensure consistency across point cloud and attribute data.
 */
uint32_t vizPointsUtils::frameID;

// -----------------------------------------------------------------------------
/**
 * @brief Stores the current position of the vehicle or reference point.
 *
 * This vector represents the 3D position (X, Y, Z) of the vehicle or a reference point
 * in the global frame. It is updated in real-time as new data is received.
 */
Eigen::Vector3f vizPointsUtils::position;

// -----------------------------------------------------------------------------
/**
 * @brief Stores the current orientation of the vehicle.
 *
 * This vector represents the roll, pitch, and yaw (RPY) orientation of the vehicle
 * in radians. It is updated in real-time as new data is received.
 */
Eigen::Vector3f vizPointsUtils::orientation;

// -----------------------------------------------------------------------------
/**
 * @brief Configuration parameters for the occupancy map.
 * 
 * @details Stores the resolution, reaching distance, and center position 
 * parameters for initializing and configuring the `OccupancyMap` instance.
 */
MapConfig mapConfig;

// -----------------------------------------------------------------------------
/**
 * @brief Configuration parameters for the cluster extractor.
 * 
 * @details Holds parameters such as tolerance, size thresholds, dynamic score 
 * thresholds, and similarity thresholds used to configure the `ClusterExtractor`.
 */
ClusterConfig clusterConfig;


// -----------------------------------------------------------------------------
// Section: initialize
// -----------------------------------------------------------------------------

void vizPointsUtils::initialize() {

    
    // Initialize the occupancyMapInstance if it hasn't been initialized
    if (!occupancyMapInstance) {
        occupancyMapInstance = std::make_unique<OccupancyMap>(
            mapConfig.resolution, mapConfig.reachingDistance, mapConfig.center
        );
    }

    // Initialize the clusterExtractorInstance if it hasn't been initialized
    if (!clusterExtractorInstance) {
        clusterExtractorInstance = std::make_unique<ClusterExtractor>(
            clusterConfig.tolerance, clusterConfig.minSize, clusterConfig.maxSize,
            clusterConfig.staticThreshold, clusterConfig.dynamicScoreThreshold,
            clusterConfig.densityThreshold, clusterConfig.velocityThreshold,
            clusterConfig.similarityThreshold, clusterConfig.maxDistanceThreshold, clusterConfig.dt
        );
    }
    
}

// -----------------------------------------------------------------------------
// Section: runOccupancyMapViewer
// -----------------------------------------------------------------------------

void vizPointsUtils::SetupTopDownView(open3d::visualization::Visualizer& vis, double cameraHeight = -8.0) {
    auto view_control = vis.GetViewControl();
    open3d::camera::PinholeCameraParameters camera_params;
    view_control.ConvertToPinholeCameraParameters(camera_params);
    camera_params.extrinsic_ <<
        1, 0, 0, 0,                  // Camera X-axis
        0, 0, 1, cameraHeight,       // Camera Y-axis (down -Z, height set by cameraHeight)
        0, -1, 0, 0,                 // Camera Z-axis (up vector)
        0, 0, 0, 1;                  // Homogeneous coordinates
    view_control.ConvertFromPinholeCameraParameters(camera_params);
}

// -----------------------------------------------------------------------------
// Section: setThreadAffinity
// -----------------------------------------------------------------------------

void vizPointsUtils::setThreadAffinity(const std::vector<int>& coreIDs, 
                                        std::mutex& consoleMutex) {
    if (coreIDs.empty()) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "Warning: No core IDs provided. Thread affinity not set.\n";
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    unsigned int maxCores = std::thread::hardware_concurrency();

    // Add specified cores to the CPU set
    for (int coreID : coreIDs) {
        if (coreID < 0 || coreID >= static_cast<int>(maxCores)) {
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cerr << "Error: Invalid core ID " << coreID << ". Skipping.\n";
            continue;
        }
        CPU_SET(coreID, &cpuset);
    }

    // Apply the CPU set to the current thread
    pthread_t thread = pthread_self();
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "Error: Failed to set thread affinity. " << std::strerror(errno) << "\n";
    } else {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cout << "Thread restricted to cores: ";
        for (int coreID : coreIDs) {
            if (coreID >= 0 && coreID < static_cast<int>(maxCores)) {
                std::cout << coreID << " ";
            }
        }
        std::cout << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Section: startListener
// -----------------------------------------------------------------------------

void vizPointsUtils::startListener(boost::asio::io_context& ioContext, 
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
                                   std::atomic<bool>& running) {
    setThreadAffinity(allowedCores,consoleMutex);

    if (host.empty() || port == 0) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "Invalid host or port specified: host='" << host << "', port=" << port << std::endl;
        return;
    }

    try {
        // Initialize the UDP socket
        UDPSocket listener(ioContext, host, port, [&](const std::vector<uint8_t>& data) {
            // {
            //     std::lock_guard<std::mutex> lock(consoleMutex);
            //     auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            //     std::cout << "[" << std::put_time(std::localtime(&now), "%F %T")
            //               << "] Received packet of size: " << data.size() << " bytes on port: " << port << std::endl;
            // }

            {
                // Protect dataAvailable with dataMutex
                std::lock_guard<std::mutex> dataLock(dataMutex);
                callbackProcessor.process(data, latestPoints);
                dataAvailable = true;
            }
            dataReadyCV.notify_one(); // Notify the processing thread

            // // DEBUG output for verification (logging)
            // {
            //     std::lock_guard<std::mutex> lock(consoleMutex); // Protect logging
            //     std::cout << "Updated Frame ID: " << latestPoints.frameID 
            //             << ", Number of Points: " << latestPoints.numVal << std::endl;
            // }

        }, bufferSize);

        {
            auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cout << "[" << std::put_time(std::localtime(&now), "%F %T") << "] Started listening on "
                      << host << ":" << port << std::endl;
        }

        // Run the io_context in the current thread
        int errorCount = 0;
        const int maxErrors = 10;

        while (running) {
            try {
                ioContext.run();
            } catch (const std::exception& e) {
                errorCount++;
                if (errorCount <= maxErrors) {
                    std::lock_guard<std::mutex> consoleLock(consoleMutex);
                    std::cerr << "Listener encountered an error: " << e.what() << ". Restarting..." << std::endl;
                }
                if (errorCount == maxErrors) {
                    std::cerr << "Error log limit reached. Suppressing further error logs.\n";
                }
                ioContext.restart();
            }
        }

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "Failed to start listener on " << host << ":" << port << ": " << e.what() << std::endl;
        return;
    }

    // Graceful shutdown
    ioContext.stop();
    {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cout << "Stopped listener on " << host << ":" << port << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Section: signalHandler
// -----------------------------------------------------------------------------

void vizPointsUtils::signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        // Set the running flag to false to signal the program to stop
        vizPointsUtils::running = false;
        
        // Notify waiting threads, in case any are blocked
        queueCV.notify_all();  

        // Log the shutdown message
        const char* message = "Shutting down listeners and processors...\n";
        ssize_t result = write(STDOUT_FILENO, message, strlen(message));
        if (result < 0) {
            // Optional: handle error in writing to STDOUT
        }
    }
}

// -----------------------------------------------------------------------------
// Section: runOccupancyMapPipeline
// -----------------------------------------------------------------------------

void vizPointsUtils::runOccupancyMapPipeline(CallbackPoints::Points& points, CallbackPoints::Points& attributes, 
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
                                             std::mutex& attributesMutex) {
    // Set thread affinity for performance optimization
    setThreadAffinity(allowedCores, consoleMutex);

    const auto targetCycleDuration = std::chrono::milliseconds(100); // Target 10 Hz processing rate

    initialize();

    while (running) {
        auto cycleStartTime = std::chrono::steady_clock::now();

        {
            std::scoped_lock lock(pointsMutex, attributesMutex, consoleMutex);

            // Validate frame synchronization and vector sizes
            if (points.frameID == attributes.frameID && points.numVal > 0 &&
                points.numVal <= points.val.size() && attributes.numVal <= attributes.val.size()) {
                
                // Extract points and attributes
                std::vector<Eigen::Vector3f> pointCloud(points.val.begin(), points.val.begin() + points.numVal);

                std::vector<float> intensity(attributes.numVal);
                std::vector<float> reflectivity(attributes.numVal);
                std::vector<float> NIR(attributes.numVal);

                // Parse attributes into separate vectors
                std::transform(attributes.val.begin(), attributes.val.begin() + attributes.numVal, intensity.begin(),
                               [](const Eigen::Vector3f& vec) { return vec.x(); });
                std::transform(attributes.val.begin(), attributes.val.begin() + attributes.numVal, reflectivity.begin(),
                               [](const Eigen::Vector3f& vec) { return vec.y(); });
                std::transform(attributes.val.begin(), attributes.val.begin() + attributes.numVal, NIR.begin(),
                               [](const Eigen::Vector3f& vec) { return vec.z(); });

                // Run the occupancy map pipeline
                occupancyMapInstance->runOccupancyMapPipeline(
                    pointCloud, intensity, reflectivity, NIR, points.NED.cast<float>(), points.frameID);

                // Process static voxels
                std::vector<OccupancyMap::VoxelData> staticVoxels_ = occupancyMapInstance->getStaticVoxels();
                // if (!staticVoxels_.empty()) {
                auto voxelColors = occupancyMapInstance->computeVoxelColors(staticVoxels_);
                staticVoxels = occupancyMapInstance->getVoxelCenters(staticVoxels_);

                occupancyColors = std::get<0>(voxelColors);
                reflectivityColors = std::get<1>(voxelColors);
                intensityColors = std::get<2>(voxelColors);
                NIRColors = std::get<3>(voxelColors);

                frameID = points.frameID;
                position = points.NED.cast<float>();
                orientation = points.RPY.cast<float>();
                // }

                // std::cout << "[OccupancyMapPipeline] Function running okay. Frame ID: " << frameID << "\n";
                std::cout << "[OccupancyMapPipeline] Function running okay. staticVoxels Size: " << staticVoxels_.size() << "\n";
            } else {
                std::cerr << "[OccupancyMapPipeline] Warning: Points and attributes are not synchronized or invalid.\n";
            }
        }

        // Timing and sleep management
        auto elapsedTime = std::chrono::steady_clock::now() - cycleStartTime;
        auto remainingSleepTime = targetCycleDuration - elapsedTime;

        if (remainingSleepTime > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(remainingSleepTime);
            {
                std::lock_guard<std::mutex> consoleLock(consoleMutex);
                std::cout << "[OccupancyMapPipeline] Processing Time: " 
                          << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count() << " ms\n";
            }
        } else {
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cout << "Warning: [OccupancyMapPipeline] Processing took longer than 100 ms. Time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count()
                      << " ms. Skipping sleep.\n";
        }
    }
}

// -----------------------------------------------------------------------------
// Section: runOccupancyMapViewer
// -----------------------------------------------------------------------------

void vizPointsUtils::runOccupancyMapViewer(uint32_t& frameID,
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
                                           std::mutex& attributesMutex) {
    setThreadAffinity(allowedCores, consoleMutex);

    const auto targetCycleDuration = std::chrono::milliseconds(100); // 10 Hz target

    TopDownViewer viewer;
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Top-Down View", 800, 800);
    vizPointsUtils::SetupTopDownView(vis, -8.0);

    while (running) {
        auto cycleStartTime = std::chrono::steady_clock::now();

        {
            std::scoped_lock lock(pointsMutex, attributesMutex, consoleMutex);

            // Validate that all data vectors have the same size
            if (staticVoxels.size() == occupancyColors.size() &&
                staticVoxels.size() == reflectivityColors.size() &&
                staticVoxels.size() == intensityColors.size() &&
                staticVoxels.size() == NIRColors.size()) {

                vis.ClearGeometries();

                // Create voxel squares and vehicle mesh
                auto static_squares = viewer.CreateVoxelSquares(staticVoxels, position, occupancyColors, mapConfig.resolution);

                // Use orientation.z() for yaw
                auto vehicle_mesh = viewer.CreateVehicleMesh(2.5f, orientation.z());

                vis.AddGeometry(static_squares);
                vis.AddGeometry(vehicle_mesh);

                std::cout << "[OccupancyMapViewer] Function running okay. Frame ID: " << frameID << "\n";

                if (!vis.PollEvents()) {  // Exit if user closes the window
                    running = false;
                }
                vis.UpdateRender();
            } else {
                std::cerr << "[OccupancyMapViewer] Error: Data vectors have inconsistent sizes.\n";
            }
        }

        // Calculate elapsed time and handle sleep
        auto elapsedTime = std::chrono::steady_clock::now() - cycleStartTime;
        auto remainingSleepTime = targetCycleDuration - elapsedTime;

        if (remainingSleepTime > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(remainingSleepTime);
            {
                std::lock_guard<std::mutex> consoleLock(consoleMutex);
                std::cout << "[OccupancyMapViewer] Processing Time: " 
                          << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count() << " ms\n";
            }
        } else {
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cout << "Warning: [OccupancyMapViewer] Processing took longer than 100 ms. Time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count()
                      << " ms. Skipping sleep.\n";
        }
    }

    // Destroy the visualizer window after exiting the loop
    vis.DestroyVisualizerWindow();
}


