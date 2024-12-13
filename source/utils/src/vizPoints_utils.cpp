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
// Section: static declaration
// -----------------------------------------------------------------------------

std::unique_ptr<OccupancyMap> vizPointsUtils::occupancyMapInstance = nullptr;

std::unique_ptr<ClusterExtractor> vizPointsUtils::clusterExtractorInstance = nullptr;

std::atomic<bool> vizPointsUtils::running = true; // Initialize the running flag to true

std::condition_variable vizPointsUtils::queueCV;  // Define the condition variable

// -----------------------------------------------------------------------------
// Section: Constructor
// -----------------------------------------------------------------------------

vizPointsUtils::vizPointsUtils() {

    
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

void vizPointsUtils::SetupTopDownView(open3d::visualization::Visualizer& vis, double cameraHeight) {
    // Set the background color to black
    vis.GetRenderOption().background_color_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Access the ViewControl
    auto& view_control = vis.GetViewControl();

    // Get the current camera parameters
    open3d::camera::PinholeCameraParameters camera_params;
    view_control.ConvertToPinholeCameraParameters(camera_params);

    // Correct extrinsics: Position the camera at (0, 0, cameraHeight) looking at the origin
    camera_params.extrinsic_ <<
        1, 0, 0, 0,                  // X-axis
        0, 1, 0, 0,                  // Y-axis
        0, 0, 1, std::abs(cameraHeight), // Z-axis (camera positioned at height)
        0, 0, 0, 1;                  // Homogeneous coordinates

    // Apply the modified camera parameters
    view_control.ConvertFromPinholeCameraParameters(camera_params);

    // Adjust zoom level
    view_control.SetZoom(1.0); // Higher value for zoom-out effect

    // Adjust near and far clipping planes
    view_control.SetConstantZNear(0.1);  // Near plane
    view_control.SetConstantZFar(10000.0); // Far plane

    // Debug: Print extrinsics and zoom to verify
    //std::cout << "Camera extrinsics applied:\n" << camera_params.extrinsic_ << std::endl;
    //std::cout << "Zoom level set to: " << view_control.GetZoom() << std::endl;

    // Force a refresh of the visualizer
    vis.PollEvents();
    vis.UpdateRender();
}


// -----------------------------------------------------------------------------
// Section: setThreadAffinity
// -----------------------------------------------------------------------------

void vizPointsUtils::setThreadAffinity(const std::vector<int>& coreIDs) {
    if (coreIDs.empty()) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "Warning: [ThreadAffinity] No core IDs provided. Thread affinity not set.\n";
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    unsigned int maxCores = std::thread::hardware_concurrency();

    // Add specified cores to the CPU set
    for (int coreID : coreIDs) {
        if (coreID < 0 || coreID >= static_cast<int>(maxCores)) {
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cerr << "Error: [ThreadAffinity] Invalid core ID " << coreID << ". Skipping.\n";
            continue;
        }
        CPU_SET(coreID, &cpuset);
    }

    // Apply the CPU set to the current thread
    pthread_t thread = pthread_self();
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "Error: [ThreadAffinity] Failed to set thread affinity. " << std::strerror(errno) << "\n";
    } else {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cout << "[ThreadAffinity] Thread restricted to cores: ";
        for (int coreID : coreIDs) {
            if (coreID >= 0 && coreID < static_cast<int>(maxCores)) {
                std::cout << coreID << " ";
            }
        }
        std::cout << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Section: startPointsListener
// -----------------------------------------------------------------------------

void vizPointsUtils::startPointsListener(boost::asio::io_context& ioContext, 
                                            const std::string& host, 
                                            uint16_t port,
                                            uint32_t bufferSize, 
                                            const std::vector<int>& allowedCores) {

    setThreadAffinity(allowedCores);
    
    if (host.empty() || port == 0) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "[PointsListener] Invalid host or port specified: host='" << host << "', port=" << port << std::endl;
        return;
    }

    try {
        // Initialize the UDP socket
        UDPSocket listener(ioContext, host, port, [&](const std::vector<uint8_t>& data) {

            // {
            //     std::lock_guard<std::mutex> consoleLock(consoleMutex);
            //     auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            //     std::cout << "[" << std::put_time(std::localtime(&now), "%F %T")
            //               << "] [PointsListener] Received packet of size: " << data.size() << " bytes on port: " << port << std::endl;
            // }

            // Swap buffers atomically
            {
                std::lock_guard<std::mutex> pLock(pointsMutex);
                callbackPointsProcessor.process(data, latestPoints);
                *P_writeBuffer = latestPoints;
                std::swap(P_writeBuffer, P_readBuffer);
                pointsDataReady = true;
                // std::cout << "[PointsListener] Data (first 10 bytes): ";
                //     for (size_t i = 0; i < std::min(data.size(), size_t(10)); ++i) {
                //         std::cout << static_cast<int>(data[i]) << " ";
                //     }
                //     std::cout << std::endl;
            }
            pointsDataReadyCV.notify_one();

        }, bufferSize);

        {
            auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cout << "[PointsListener] [" << std::put_time(std::localtime(&now), "%F %T") << "] Started listening on "
                      << host << ":" << port << std::endl;
        }

        // Run the io_context in the current thread
        int errorCount = 0;
        const int maxErrors = 5;

        while (vizPointsUtils::running) {
            try {
                ioContext.run();
            } catch (const std::exception& e) {
                errorCount++;
                if (errorCount <= maxErrors) {
                    std::lock_guard<std::mutex> consoleLock(consoleMutex);
                    std::cerr << "[PointsListener] Listener encountered an error: " << e.what() << ". Restarting..." << std::endl;
                }
                if (errorCount == maxErrors) {
                    std::cerr << "[PointsListener] Error log limit reached. Suppressing further error logs.\n";
                }
                ioContext.restart();
            }
        }

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "[PointsListener] Failed to start listener on " << host << ":" << port << ": " << e.what() << std::endl;
        return;
    }

    // Graceful shutdown
    ioContext.stop();
    {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cout << "[PointsListener] Stopped listener on " << host << ":" << port << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Section: startAttributeListener
// -----------------------------------------------------------------------------

void vizPointsUtils::startAttributesListener(boost::asio::io_context& ioContext, 
                                                const std::string& host, 
                                                uint16_t port,
                                                uint32_t bufferSize, 
                                                const std::vector<int>& allowedCores) {

    setThreadAffinity(allowedCores);

    if (host.empty() || port == 0) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "[AttributesListener] Invalid host or port specified: host='" << host << "', port=" << port << std::endl;
        return;
    }

    try {
        // Initialize the UDP socket
        UDPSocket listener(ioContext, host, port, [&](const std::vector<uint8_t>& data) {

            // {
            //     std::lock_guard<std::mutex> consoleLock(consoleMutex);
            //     auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            //     std::cout << "[" << std::put_time(std::localtime(&now), "%F %T")
            //               << "] [AttributesListener] Received packet of size: " << data.size() << " bytes on port: " << port << std::endl;
            // }

            // Swap buffers atomically
            {
                std::lock_guard<std::mutex> aLock(attributesMutex);
                callbackAttributesProcessor.process(data, latestAttributes);
                *A_writeBuffer = latestAttributes;
                std::swap(A_writeBuffer, A_readBuffer);
                attributesDataReady = true;
                // std::cout << "[AttributesListener] Data (first 10 bytes): ";
                //     for (size_t i = 0; i < std::min(data.size(), size_t(10)); ++i) {
                //         std::cout << static_cast<int>(data[i]) << " ";
                //     }
                //     std::cout << std::endl;
                // std::cout << "[latestAttributes] numVal :" << latestAttributes.numVal << std::endl;
                // std::cout << "[A_writeBuffer] numVal :" << A_writeBuffer->numVal << std::endl;
            }
            attributesDataReadyCV.notify_one();

        }, bufferSize);

        {
            auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::lock_guard<std::mutex> consoleLock(consoleMutex);
            std::cout << "[AttributesListener] [" << std::put_time(std::localtime(&now), "%F %T") << "] Started listening on "
                      << host << ":" << port << std::endl;
        }

        // Run the io_context in the current thread
        int errorCount = 0;
        const int maxErrors = 5;

        while (vizPointsUtils::running) {
            try {
                ioContext.run();
            } catch (const std::exception& e) {
                errorCount++;
                if (errorCount <= maxErrors) {
                    std::lock_guard<std::mutex> consoleLock(consoleMutex);
                    std::cerr << "[AttributesListener] Listener encountered an error: " << e.what() << ". Restarting..." << std::endl;
                }
                if (errorCount == maxErrors) {
                    std::cerr << "[AttributesListener] Error log limit reached. Suppressing further error logs.\n";
                }
                ioContext.restart();
            }
        }

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cerr << "[AttributesListener] Failed to start listener on " << host << ":" << port << ": " << e.what() << std::endl;
        return;
    }

    // Graceful shutdown
    ioContext.stop();
    {
        std::lock_guard<std::mutex> consoleLock(consoleMutex);
        std::cout << "[AttributesListener] Stopped listener on " << host << ":" << port << std::endl;
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
        vizPointsUtils::queueCV.notify_all();  

        // Log the shutdown message
        const char* message = "[signalHandler] Shutting down listeners and processors...\n";
        ssize_t result = write(STDOUT_FILENO, message, strlen(message));
        if (result < 0) {
            // Optional: handle error in writing to STDOUT
        }
    }
}

// -----------------------------------------------------------------------------
// Section: runOccupancyMapPipeline
// -----------------------------------------------------------------------------

void vizPointsUtils::runOccupancyMapPipeline(const std::vector<int>& allowedCores) {
    // Set thread affinity for performance optimization
    setThreadAffinity(allowedCores);

    const auto targetCycleDuration = std::chrono::milliseconds(150); // Target 10 Hz processing rate

    while (vizPointsUtils::running) {
        auto cycleStartTime = std::chrono::steady_clock::now();

        CallbackPoints::Points localPointsBuffer;
        CallbackPoints::Points localAttributessBuffer;

        {   
            // Wait for new data to be ready
            std::unique_lock<std::mutex> pLock(pointsMutex);
            pointsDataReadyCV.wait(pLock, [this] { return this->pointsDataReady.load(); });
            localPointsBuffer = *P_readBuffer; // Copy the read buffer
            pointsDataReady = false; // Reset dataReady flag
            // std::cout << "[P_readBuffer] frameID :" << P_readBuffer->frameID << " numVal :" << P_readBuffer->numVal << std::endl;
        }

        {   
            // Wait for new data to be ready
            std::unique_lock<std::mutex> attLock(attributesMutex);
            attributesDataReadyCV.wait(attLock, [this] { return this->attributesDataReady.load(); });
            localAttributessBuffer = *A_readBuffer; // Copy the read buffer
            attributesDataReady = false; // Reset dataReady flag
            // std::cout << "[A_readBuffer] frameID :" << A_readBuffer->frameID << " numVal :" << A_readBuffer->numVal << std::endl;
        }

        std::optional<std::pair<CallbackPoints::Points, CallbackPoints::Points>> matchedData;

        {
            std::lock_guard<std::mutex> dequeLock(dequeMutex);

            // Add new data to the deque
            pointsDeque.push_back(localPointsBuffer);
            attributesDeque.push_back(localAttributessBuffer);

            // Keep the size of each deque to the last 3 entries
            if (pointsDeque.size() > 3) pointsDeque.pop_front();
            if (attributesDeque.size() > 3) attributesDeque.pop_front();

            // Find the latest matching frame ID
            for (auto itPoints = pointsDeque.rbegin(); itPoints != pointsDeque.rend(); ++itPoints) {
                for (auto itAttributes = attributesDeque.rbegin(); itAttributes != attributesDeque.rend(); ++itAttributes) {
                    if (itPoints->frameID == itAttributes->frameID) {
                        matchedData = std::make_pair(*itPoints, *itAttributes); // Store the match
                        break; // Exit inner loop
                    }
                }
                if (matchedData.has_value()) break; // Exit outer loop
            }
        }

        OccupancyMapData localBuffer;

        if (matchedData.has_value()) {

            // Access the matched data
            const auto& [matchedPoints, matchedAttributes] = matchedData.value();

            // Retrieve the first (PointsData) and second (AttributesData) elements
            const CallbackPoints::Points& points = matchedPoints;     // First element
            const CallbackPoints::Points& attributes = matchedAttributes; // Second element

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
            occupancyMapInstance->runOccupancyMapPipeline(pointCloud, 
                                                            intensity, 
                                                            reflectivity, 
                                                            NIR, 
                                                            points.NED.cast<float>(), 
                                                            points.frameID);

            // Process static voxels
            std::vector<OccupancyMap::VoxelData> staticVoxels_ = occupancyMapInstance->getStaticVoxels();
            if (!staticVoxels_.empty()) {
                auto voxelColors = occupancyMapInstance->computeVoxelColors(staticVoxels_);
                localBuffer.staticVoxels = occupancyMapInstance->getVoxelCenters(staticVoxels_);

                localBuffer.occupancyColors = std::get<0>(voxelColors);
                localBuffer.reflectivityColors = std::get<1>(voxelColors);
                localBuffer.intensityColors = std::get<2>(voxelColors);
                localBuffer.NIRColors = std::get<3>(voxelColors);

                localBuffer.frameID = points.frameID;
                localBuffer.position = points.NED.cast<float>();
                localBuffer.orientation = points.RPY.cast<float>();

                // Swap buffers atomically
                {
                    std::lock_guard<std::mutex> occMapLock(occupancyMapDataMutex);
                    *OMD_writeBuffer = localBuffer;
                    std::swap(OMD_writeBuffer, OMD_readBuffer);
                    occupancyMapDataReady = true;
                }
                occupancyMapDataReadyCV.notify_one();
            }

            //std::cout << "[OccupancyMapPipeline] Function running okay. Frame ID: " << OMD_readBuffer->staticVoxels.size() << "\n";

        } else {

            std::cerr << "[OccupancyMapPipeline] Warning: Points and attributes are not synchronized or invalid.\n";
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

void vizPointsUtils::runOccupancyMapViewer(const std::vector<int>& allowedCores, open3d::visualization::Visualizer& vis) {
    // Set thread affinity for optimal core usage
    setThreadAffinity(allowedCores);

    // Define target cycle duration (200ms = 5Hz)
    const auto targetCycleDuration = std::chrono::milliseconds(150);

    TopDownViewer viewer;

    // Main loop for rendering and updating the viewer
    while (vizPointsUtils::running) {
        auto cycleStartTime = std::chrono::steady_clock::now();

        OccupancyMapData localBuffer;

        {
            // Wait for new data to be ready
            std::unique_lock<std::mutex> occMapLock(occupancyMapDataMutex);
            occupancyMapDataReadyCV.wait(occMapLock, [this] { return this->occupancyMapDataReady.load(); });
            localBuffer = *OMD_readBuffer; // Copy the read buffer
            occupancyMapDataReady = false; // Reset dataReady flag
        }

        // Clear existing geometries
        try {
            vis.ClearGeometries();
        } catch (const std::exception& e) {
            std::cerr << "Error clearing geometries: " << e.what() << std::endl;
        }


        // Create voxel squares and the vehicle mesh
        auto static_squares = viewer.CreateVoxelSquares(localBuffer.staticVoxels, 
                                                        localBuffer.position, 
                                                        localBuffer.occupancyColors, 
                                                        mapConfig.resolution);

        auto vehicle_mesh = viewer.CreateVehicleMesh(mapConfig.resolution, localBuffer.orientation.z());

        // Add geometries to the visualizer
        if (!vis.AddGeometry(static_squares)) {
            std::cerr << "Error: [OccupancyMapViewer] Failed to add static squares.\n";
            vizPointsUtils::running = false; // Exit if adding fails
            break;
        }

        if (!vis.AddGeometry(vehicle_mesh)) {
            std::cerr << "Error: [OccupancyMapViewer] Failed to add vehicle mesh.\n";
            vizPointsUtils::running = false; // Exit if adding fails
            break;
        }

         if (!vis.PollEvents()) {
            vizPointsUtils::running = false; // Exit if the user closes the window
            break;
        }

        try {
            vis.UpdateRender();
        } catch (const std::exception& e) {
            std::cerr << "Error in rendering: " << e.what() << std::endl;
            vizPointsUtils::running = false;
        }

        // Handle sleep and ensure consistent frame rate
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
            std::cerr << "Warning: [OccupancyMapViewer] Processing took longer than the target cycle duration.\n";
        }
    }

    // Cleanly destroy the visualizer window before exiting
    vis.DestroyVisualizerWindow();
}



