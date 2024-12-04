#include "udpSocket.hpp"
#include "callbackPoints.hpp"
#include "occupancyMap.hpp"
#include "clusterExtractor.hpp"
#include "top_down_viewer.hpp"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <condition_variable>

std::mutex consoleMutex;

static std::unique_ptr<OccupancyMap> occupancyMapInstance = nullptr;
static std::unique_ptr<ClusterExtractor> clusterExtractorInstance = nullptr;

// Logging utility
void logMessage(const std::string& message) {
    std::scoped_lock lock(consoleMutex);
    std::cout << message << std::endl;
}

void logError(const std::string& errorMessage) {
    std::scoped_lock lock(consoleMutex);
    std::cerr << "[ERROR] " << errorMessage << std::endl;
}

// Listener function
void startListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port,
                   const std::function<void(const std::vector<uint8_t>&)>& callback, uint32_t bufferSize, std::atomic<bool>& running) {
    try {
        UDPSocket listener(ioContext, host, port, callback, bufferSize);
        logMessage("Started listening on " + host + ":" + std::to_string(port));

        while (running) {
            ioContext.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ioContext.stop();
    } catch (const std::exception& e) {
        logError("Exception in UDP listener: " + std::string(e.what()));
    }
}


// Setup top-down visualization view
void SetupTopDownView(open3d::visualization::Visualizer& vis, double cameraHeight = -8.0) {
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

int main() {
    logMessage("Initializing application...");

    // Shared resources
    std::mutex pointsMutex, attributesMutex,occupancyMapMutex;
    CallbackPoints::Points points;
    CallbackPoints pointsProcessor;
    CallbackPoints::Points attributes;
    CallbackPoints attributesProcessor;

    // Atomic flags
    std::atomic<bool> running(true);
    std::atomic<bool> dataReady(false);

    // Condition variable for processing thread
    std::condition_variable dataCondition;

    // Configurations
    MapConfig mapConfig;
    ClusterConfig clusterConfig;

    try {
        // Start UDP listeners
        std::vector<std::thread> threads;

        // Static voxel listener
        boost::asio::io_context ioContextPoints;
        auto udpCallbackPoints = [&pointsProcessor, &points, &pointsMutex, &dataReady, &dataCondition](const std::vector<uint8_t>& data) {
            {
                std::scoped_lock lock(pointsMutex);
                pointsProcessor.process(data, points);
            }
            dataReady = true;
            dataCondition.notify_one();
            logMessage("Received data on points listener. Size: " + std::to_string(data.size()));
        };
        threads.emplace_back(startListener, std::ref(ioContextPoints), "192.168.1.10", 55000, udpCallbackPoints, 1393, std::ref(running));

        // Dynamic voxel listener
        boost::asio::io_context ioContextAttributes;
        auto udpCallbackAttributes = [&attributesProcessor, &attributes, &attributesMutex, &dataReady, &dataCondition](const std::vector<uint8_t>& data) {
            {
                std::scoped_lock lock(attributesMutex);
                attributesProcessor.process(data, attributes);
            }
            dataReady = true;
            dataCondition.notify_one();
            logMessage("Received data on attributes listener. Size: " + std::to_string(data.size()));
        };
        threads.emplace_back(startListener, std::ref(ioContextAttributes), "192.168.1.10", 55001, udpCallbackAttributes, 1393, std::ref(running));
    
        std::thread processingThread([&]() {
            while (running) {
                // Lock pointsMutex and attributesMutex together to avoid deadlock
                std::unique_lock<std::mutex> pointsLock(pointsMutex);
                std::unique_lock<std::mutex> attributesLock(attributesMutex);
                
                // Wait for data readiness or termination signal
                dataCondition.wait(pointsLock, [&]() { return dataReady || !running; });

                if (!running) break;

                try {
                    // Initialize instances if not already done
                    if (!occupancyMapInstance || !clusterExtractorInstance) {
                        std::scoped_lock occupancyLock(occupancyMapMutex); // Lock during initialization
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

                    // Parse attributes (this doesn't need occupancyMapMutex)
                    std::vector<float> intensity, reflectivity, NIR;
                    intensity.reserve(attributes.val.size());
                    reflectivity.reserve(attributes.val.size());
                    NIR.reserve(attributes.val.size());

                    for (const auto& vec : attributes.val) {
                        intensity.push_back(vec.x());
                        reflectivity.push_back(vec.y());
                        NIR.push_back(vec.z());
                    }

                    // Run processing pipelines (lock occupancyMapInstance for safety)
                    {
                        std::scoped_lock occupancyLock(occupancyMapMutex); // Ensure thread safety
                        if (!points.val.empty()){
                            clusterExtractorInstance->runClusterExtractorPipeline(points.val, intensity, reflectivity, NIR);
                            auto dynamicCloud = clusterExtractorInstance->getDynamicClusterPoints();
                            occupancyMapInstance->runOccupancyMapPipeline(points.val, intensity, reflectivity, NIR, dynamicCloud, points.NED.cast<float>(), points.frameID);
                        }
                    }

                    // Reset the data ready flag
                    dataReady = false;
                } catch (const std::exception& e) {
                    logError("Processing thread exception: " + std::string(e.what()));
                }
            }
        });

        // // Visualization
        // TopDownViewer viewer;
        // open3d::visualization::Visualizer vis;
        // vis.CreateVisualizerWindow("Top-Down View", 800, 800);
        // SetupTopDownView(vis, -8.0);

        // while (running) {
        //     {
        //         // Lock points and attributes mutex only when accessing these resources
        //         std::scoped_lock dataLock(pointsMutex, attributesMutex);

        //         auto vehiclePos = points.NED.cast<float>();
        //         uint32_t currFrame = points.frameID;
        //         double yaw_rad = points.RPY(2);

        //         {
        //             // Lock occupancyMapMutex only when accessing occupancyMapInstance
        //             std::scoped_lock occupancyLock(occupancyMapMutex);

        //             // Clear previous geometry only if needed
        //             vis.ClearGeometries();

        //             if (occupancyMapInstance) {
        //                 // Handle static voxels
        //                 if (auto staticVoxel = occupancyMapInstance->getStaticVoxels(); !staticVoxel.empty()) {
        //                     auto voxelColors = occupancyMapInstance->computeVoxelColors(staticVoxel);
        //                     auto staticVoxelVec = occupancyMapInstance->getVoxelCenters(staticVoxel);
        //                     const auto& occupancyColors = std::get<0>(voxelColors);
        //                     std::vector<Eigen::Matrix<uint32_t, 3, 1>> occColors;
        //                     occColors.reserve(occupancyColors.size());
        //                     std::transform(occupancyColors.begin(), occupancyColors.end(), std::back_inserter(occColors), [](const Eigen::Vector3i& vec) {
        //                         return vec.cwiseMin(255).cast<uint32_t>();  // Clamp to avoid overflow
        //                     });
        //                     auto static_squares = viewer.CreateVoxelSquares(staticVoxelVec, vehiclePos, occColors, mapConfig.resolution);
        //                     vis.AddGeometry(static_squares);
        //                 }

        //                 // Handle dynamic voxels
        //                 if (auto dynamicVoxel = occupancyMapInstance->getDynamicVoxels(); !dynamicVoxel.empty()) {
        //                     auto dynamicVoxelVec = occupancyMapInstance->getVoxelCenters(dynamicVoxel);
        //                     auto dynamicVoxelColor = occupancyMapInstance->assignVoxelColorsRed(dynamicVoxel);
        //                     std::vector<Eigen::Matrix<uint32_t, 3, 1>> dynColors;
        //                     dynColors.reserve(dynamicVoxelColor.size());
        //                     std::transform(dynamicVoxelColor.begin(), dynamicVoxelColor.end(), std::back_inserter(dynColors), [](const Eigen::Vector3i& vec) {
        //                         return vec.cwiseMin(255).cast<uint32_t>();  // Clamp to avoid overflow
        //                     });
        //                     auto dynamic_squares = viewer.CreateVoxelSquares(dynamicVoxelVec, vehiclePos, dynColors, mapConfig.resolution);
        //                     vis.AddGeometry(dynamic_squares);
        //                 }
        //             }
        //         }

        //         // Add vehicle mesh
        //         auto vehicle_mesh = viewer.CreateVehicleMesh(2.5f, yaw_rad);
        //         vis.AddGeometry(vehicle_mesh);
        //     }

        //     if (!vis.PollEvents()) {  // Exit if user closes the window
        //         running = false;
        //         break;
        //     }

        //     vis.UpdateRender();
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }

        // vis.DestroyVisualizerWindow();

        // Cleanup
        running = false;
        dataCondition.notify_all();
        if (processingThread.joinable()) processingThread.join();
        for (auto& thread : threads) {
            if (thread.joinable()) thread.join();
        }

    } catch (const std::exception& e) {
        logError("Exception in main: " + std::string(e.what()));
        running = false;
        return EXIT_FAILURE;
    }
    logMessage("Ending application...");
    return EXIT_SUCCESS;
}

