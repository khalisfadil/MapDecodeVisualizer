#include "udpSocket.hpp"
#include "callbackPoints.hpp"
#include "occupancyMap.hpp"
#include "clusterExtractor.hpp"

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

// ########################################################
// Global variables
std::mutex consoleMutex;   // Synchronize console output
std::atomic<bool> running(true);  // Atomic flag for program termination
std::mutex pointsMutex;    // Mutex for protecting points data
std::mutex attributesMutex;  // Mutex for protecting attributes data
std::mutex occupancyMapMutex;
std::condition_variable queueCV;  // Condition variable for signaling
MapConfig mapConfig;
ClusterConfig clusterConfig;


// ########################################################
// shared across all instances of the class
static std::unique_ptr<OccupancyMap> occupancyMapInstance = nullptr;
static std::unique_ptr<ClusterExtractor> clusterExtractorInstance = nullptr;

// ########################################################
// Function to handle signal for graceful shutdown
void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        running = false;
        queueCV.notify_all();  // Wake up processing threads

        const char* message = "Shutting down listeners and processors...\n";
        ssize_t result = write(STDOUT_FILENO, message, strlen(message));
        if (result < 0) {
            // Optional: Log an error or take action (unlikely necessary in a signal handler)
        }
    }
}
// ########################################################
// Function to set thread affinity
void setThreadAffinity(const std::vector<int>& coreIDs) {
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
// ########################################################
// Function to start a UDP listener in a separate thread
void startListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port,
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
            {
                std::lock_guard<std::mutex> lock(consoleMutex);
                auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                std::cout << "[" << std::put_time(std::localtime(&now), "%F %T")
                          << "] Received packet of size: " << data.size() << " bytes on port: " << port << std::endl;
            }

            {   
                // Process the packet
                std::lock_guard<std::mutex> lock(dataMutex);
                callbackProcessor.process(data, latestPoints);

                {
                    std::lock_guard<std::mutex> cvLock(consoleMutex); // Ensure thread-safety
                    dataAvailable = true;
                }
                dataReadyCV.notify_one(); // Notify the processing thread

                // Debug output for verification
                std::cout << "Updated Frame ID: " << latestPoints.frameID 
                          << ", Number of Points: " << latestPoints.numVal << std::endl;
            }
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
// ########################################################
// Processing function to consume data at a fixed rate (10 Hz)
void pointToWorkWith(CallbackPoints::Points& points, CallbackPoints::Points& attributes,
                     const std::vector<int>& allowedCores) {
    setThreadAffinity(allowedCores);

    const auto targetCycleDuration = std::chrono::milliseconds(100); // 10 Hz target

    while (running) {
        auto cycleStartTime = std::chrono::steady_clock::now();

        {
            std::scoped_lock lock(pointsMutex, attributesMutex, consoleMutex, occupancyMapMutex);
            // Process points and attributes
            if (points.frameID == attributes.frameID && points.numVal > 0) {

                // Parse attributes efficiently
                std::vector<float> intensity(attributes.val.size());
                std::vector<float> reflectivity(attributes.val.size());
                std::vector<float> NIR(attributes.val.size());

                std::transform(attributes.val.begin(), attributes.val.end(), intensity.begin(),
                            [](const auto& vec) { return vec.x(); });
                std::transform(attributes.val.begin(), attributes.val.end(), reflectivity.begin(),
                            [](const auto& vec) { return vec.y(); });
                std::transform(attributes.val.begin(), attributes.val.end(), NIR.begin(),
                            [](const auto& vec) { return vec.z(); });

                clusterExtractorInstance->runClusterExtractorPipeline(points.val, intensity, reflectivity, NIR);
                auto dynamicCloud = clusterExtractorInstance->getDynamicClusterPoints();
                occupancyMapInstance->runOccupancyMapPipeline(points.val, intensity, reflectivity, NIR, dynamicCloud, points.NED.cast<float>(), points.frameID);
                
                auto dynamicVoxelVector = occupancyMapInstance->getDynamicVoxels();

                std::cout << "Function running okay.\n";
                std::cout << "dynamicVoxelVector. Size: " 
                      << dynamicVoxelVector.size()
                      << "\n";
            }
        }

        // Calculate elapsed time and handle sleep
        auto elapsedTime = std::chrono::steady_clock::now() - cycleStartTime;
        auto remainingSleepTime = targetCycleDuration - elapsedTime;

        if (remainingSleepTime > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(remainingSleepTime);
        } else {
            std::lock_guard<std::mutex> lock(consoleMutex);
            std::cout << "Warning: Processing took longer than 100 ms. Time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count()
                      << " ms. Skipping sleep.\n";
        }
    }
}
// ########################################################
// Main initializeSharedResources
void initializeSharedResources() {
    if (!occupancyMapInstance) {
        occupancyMapInstance = std::make_unique<OccupancyMap>(mapConfig.resolution, mapConfig.reachingDistance, mapConfig.center);
    }
    if (!clusterExtractorInstance) {
        clusterExtractorInstance = std::make_unique<ClusterExtractor>(
            clusterConfig.tolerance, clusterConfig.minSize, clusterConfig.maxSize,
            clusterConfig.staticThreshold, clusterConfig.dynamicScoreThreshold,
            clusterConfig.densityThreshold, clusterConfig.velocityThreshold,
            clusterConfig.similarityThreshold, clusterConfig.maxDistanceThreshold, clusterConfig.dt);
    }
}

// ########################################################
// Main Function
int main() {
    // Register signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, nullptr);
    sigaction(SIGTERM, &sigIntHandler, nullptr);

    std::cout << "Listening to incoming UDP packets..." << std::endl;

    // Shared resources
    CallbackPoints::Points points_, attributes_;
    CallbackPoints callbackPoints_, callbackAttributes_;

    // Synchronization primitives
    std::condition_variable dataReadyCV;
    std::atomic<bool> dataAvailable(false);

    initializeSharedResources();  // Encapsulated lazy initialization

    try {
        std::vector<std::thread> threads;

        // Listener configurations
        std::string pointsHost = "127.0.0.1";
        uint16_t pointsPort = 61234;
        std::string attributesHost = "127.0.0.1";
        uint16_t attributesPort = 61235;

        // Start points Listener
        boost::asio::io_context ioContextPoints;
        threads.emplace_back(
            startListener, 
            std::ref(ioContextPoints),         // Pass io_context by reference
            pointsHost,                        // Copy string (simple type)
            pointsPort,                        // Copy port number (simple type)
            1393,                              // Copy bufferSize (simple type)
            std::vector<int>{8},               // Pass vector (temporary, can be moved)
            std::ref(callbackPoints_),         // Pass callbackProcessor by reference
            std::ref(points_),                 // Pass latestPoints by reference
            std::ref(pointsMutex),             // Pass mutex by reference
            std::ref(dataReadyCV),             // Pass condition_variable by reference
            std::ref(dataAvailable)            // Pass atomic<bool> by reference
        );

        // Start attributes Listener
        boost::asio::io_context ioContextAttributes;
        threads.emplace_back(
            startListener, 
            std::ref(ioContextAttributes),     // Pass io_context by reference
            attributesHost,                    // Copy string (simple type)
            attributesPort,                    // Copy port number (simple type)
            1393,                              // Copy bufferSize (simple type)
            std::vector<int>{9},               // Pass vector (temporary, can be moved)
            std::ref(callbackAttributes_),     // Pass callbackProcessor by reference
            std::ref(attributes_),             // Pass latestPoints by reference
            std::ref(attributesMutex),         // Pass mutex by reference
            std::ref(dataReadyCV),             // Pass condition_variable by reference
            std::ref(dataAvailable)            // Pass atomic<bool> by reference
        );

        // Start Processing (10 Hz)
        threads.emplace_back([&]() {
            pointToWorkWith(std::ref(points_), std::ref(attributes_), std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7});
        });

        // Monitor signal and clean up
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop IO contexts
        ioContextPoints.stop();
        ioContextAttributes.stop();

        // Join all threads
        for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "All listeners stopped. Exiting program." << std::endl;
    return EXIT_SUCCESS;
}


