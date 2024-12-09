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
 * @brief Mutex for synchronizing access to the occupancy map.
 * 
 * @details Protects the shared occupancy map data structure, ensuring 
 * consistency and thread-safety during concurrent access and updates.
 */
std::mutex vizPointsUtils::occupancyMapMutex;

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
// Section: setThreadAffinity
// -----------------------------------------------------------------------------

void vizPointsUtils::setThreadAffinity(const std::vector<int>& coreIDs) {
    if (coreIDs.empty()) {
        std::lock_guard<std::mutex> lock(consoleMutex);
        std::cerr << "Warning: No core IDs provided. Thread affinity not set.\n";
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    unsigned int maxCores = std::thread::hardware_concurrency();

    // Add specified cores to the CPU set
    for (int coreID : coreIDs) {
        if (coreID < 0 || coreID >= static_cast<int>(maxCores)) {
            std::lock_guard<std::mutex> lock(consoleMutex);
            std::cerr << "Error: Invalid core ID " << coreID << ". Skipping.\n";
            continue;
        }
        CPU_SET(coreID, &cpuset);
    }

    // Apply the CPU set to the current thread
    pthread_t thread = pthread_self();
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::lock_guard<std::mutex> lock(consoleMutex);
        std::cerr << "Error: Failed to set thread affinity. " << std::strerror(errno) << "\n";
    } else {
        std::lock_guard<std::mutex> lock(consoleMutex);
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

void vizPointsUtils::startListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port,
                                   uint32_t bufferSize, const std::vector<int>& allowedCores,
                                   CallbackPoints& callbackProcessor, CallbackPoints::Points& latestPoints,
                                   std::mutex& dataMutex, std::condition_variable& dataReadyCV,
                                   std::atomic<bool>& dataAvailable) {
    setThreadAffinity(allowedCores);

    if (host.empty() || port == 0) {
        std::lock_guard<std::mutex> lock(consoleMutex);
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
                std::lock_guard<std::mutex> lock(dataMutex);
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
            std::lock_guard<std::mutex> lock(consoleMutex);
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
                    std::lock_guard<std::mutex> lock(consoleMutex);
                    std::cerr << "Listener encountered an error: " << e.what() << ". Restarting..." << std::endl;
                }
                if (errorCount == maxErrors) {
                    std::cerr << "Error log limit reached. Suppressing further error logs.\n";
                }
                ioContext.restart();
            }
        }

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(consoleMutex);
        std::cerr << "Failed to start listener on " << host << ":" << port << ": " << e.what() << std::endl;
        return;
    }

    // Graceful shutdown
    ioContext.stop();
    {
        std::lock_guard<std::mutex> lock(consoleMutex);
        std::cout << "Stopped listener on " << host << ":" << port << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Section: signalHandler
// -----------------------------------------------------------------------------

void vizPointsUtils::signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        // Set the running flag to false to signal the program to stop
        running = false;
        
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
// Section: pointToWorkWith
// -----------------------------------------------------------------------------

void vizPointsUtils::pointToWorkWith(CallbackPoints::Points& points, CallbackPoints::Points& attributes,
                     const std::vector<int>& allowedCores) {
    setThreadAffinity(allowedCores);

    const auto targetCycleDuration = std::chrono::milliseconds(100); // 10 Hz target

    initialize();

    while (running) {
        auto cycleStartTime = std::chrono::steady_clock::now();

        {
            std::scoped_lock lock(pointsMutex, attributesMutex, consoleMutex, occupancyMapMutex);
            // Process points and attributes
            if (points.frameID == attributes.frameID && points.numVal > 0) {
                // Extract a subset of points from points.val
                std::vector<Eigen::Vector3f> pointCloud(points.val.begin(), points.val.begin() + points.numVal);

                // Pre-size attribute vectors
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

                occupancyMapInstance->runOccupancyMapPipeline(
                    pointCloud, intensity, reflectivity, NIR, points.NED.cast<float>(), points.frameID);

                // Debugging output
                std::cout << "Function running okay. Frame ID: " << points.frameID << "\n";
                // std::cout << "static Size: " << staticVoxelVector.size() << "\n";
            }
        }

        // Calculate elapsed time and handle sleep
        auto elapsedTime = std::chrono::steady_clock::now() - cycleStartTime;
        auto remainingSleepTime = targetCycleDuration - elapsedTime;

        if (remainingSleepTime > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(remainingSleepTime);
            std::cout << "Processing Time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count()<< "\n";;
        } else {
            std::lock_guard<std::mutex> lock(consoleMutex);
            std::cout << "Warning: Processing took longer than 100 ms. Time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count()
                      << " ms. Skipping sleep.\n";
        }
    }
}