#include "udpSocket.hpp"
#include "callbackStaticVoxel.hpp"
#include "callbackDynamicVoxel.hpp"
#include "callbackDynamicVoxelColor.hpp"
#include "callbackStaticVoxelColor.hpp"
#include "top_down_viewer.hpp"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <mutex>

std::mutex consoleMutex;

// Function to start a UDP listener in a separate thread
void startListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port,
                   const std::function<void(const std::vector<uint8_t>&)>& callback, uint32_t bufferSize) {
    // Initialize the UDP socket
    UDPSocket listener(ioContext, host, port, callback, bufferSize);

    // Synchronize printing to the console
    {
        std::lock_guard<std::mutex> lock(consoleMutex);
        std::cout << "Started listened on " << host << ":" << port << std::endl;
    }

    // Run the io_context in the current thread
    ioContext.run();
}


int main() {

    std::cout << "Listening to incoming UDP packets......" << std::endl;
    bool success = false;

    // Shared resources with mutex protection
    std::mutex staticVoxelMutex, dynamicVoxelMutex, staticVoxelColorMutex, dynamicVoxelColorMutex;

    CallbackStaticVoxel::Voxel staticVoxel_;
    CallbackStaticVoxel CallbackStaticVoxel_;
    CallbackDynamicVoxel::Voxel dynamicVoxel_;
    CallbackDynamicVoxel CallbackDynamicVoxel_;
    CallbackStaticVoxelColor::Voxel staticVoxelColor_;
    CallbackStaticVoxelColor CallbackStaticVoxelColor_;
    CallbackDynamicVoxelColor::Voxel dynamicVoxelColor_;
    CallbackDynamicVoxelColor CallbackDynamicVoxelColor_;

    try {
        // Thread container
        std::vector<std::thread> threads;

        // ###################################################################
        // Static Voxel Listener
        boost::asio::io_context ioContextStaticVoxel;
        auto udpCallbackStaticVoxel = [&CallbackStaticVoxel_, &staticVoxel_, &staticVoxelMutex](const std::vector<uint8_t>& data) {
            std::lock_guard<std::mutex> lock(staticVoxelMutex);
            CallbackStaticVoxel_.process(data, staticVoxel_);
        };
        threads.emplace_back(startListener, std::ref(ioContextStaticVoxel), "139.30.200.74", 49152, udpCallbackStaticVoxel, 1447);

        // ###################################################################
        // Dynamic Voxel Listener
        boost::asio::io_context ioContextDynamicVoxel;
        auto udpCallbackDynamicVoxel = [&CallbackDynamicVoxel_, &dynamicVoxel_, &dynamicVoxelMutex](const std::vector<uint8_t>& data) {
            std::lock_guard<std::mutex> lock(dynamicVoxelMutex);
            CallbackDynamicVoxel_.process(data, dynamicVoxel_);
        };
        threads.emplace_back(startListener, std::ref(ioContextDynamicVoxel), "139.30.200.74", 49153, udpCallbackDynamicVoxel, 1447);

        // ###################################################################
        // Static Voxel Color Listener
        boost::asio::io_context ioContextStaticVoxelColor;
        auto udpCallbackStaticVoxelColor = [&CallbackStaticVoxelColor_, &staticVoxelColor_, &staticVoxelColorMutex](const std::vector<uint8_t>& data) {
            std::lock_guard<std::mutex> lock(staticVoxelColorMutex);
            CallbackStaticVoxelColor_.process(data, staticVoxelColor_);
        };
        threads.emplace_back(startListener, std::ref(ioContextStaticVoxelColor), "139.30.200.74", 49154, udpCallbackStaticVoxelColor, 1447);

        // ###################################################################
        // Dynamic Voxel Color Listener
        boost::asio::io_context ioContextDynamicVoxelColor;
        auto udpCallbackDynamicVoxelColor = [&CallbackDynamicVoxelColor_, &dynamicVoxelColor_, &dynamicVoxelColorMutex](const std::vector<uint8_t>& data) {
            std::lock_guard<std::mutex> lock(dynamicVoxelColorMutex);
            CallbackDynamicVoxelColor_.process(data, dynamicVoxelColor_);
        };
        threads.emplace_back(startListener, std::ref(ioContextDynamicVoxelColor),"139.30.200.74", 49155, udpCallbackDynamicVoxelColor, 1447);

        // ###################################################################
        // Wait for all threads to finish
        for (auto& thread : threads) {
            thread.join();
        }

        success = true;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        success = false;
    }

    // ###################################################################
    // Continue execution based on success or failure
    if (success) {
        std::cout << "All listeners started successfully. Continuing program execution..." << std::endl;

        // Display the active UDP ports
        std::cout << "Currently listening on the following UDP ports:" << std::endl;
        std::cout << "1. 139.30.200.74:49152 - Static Voxel" << std::endl;
        std::cout << "2. 139.30.200.74:49153 - Dynamic Voxel" << std::endl;
        std::cout << "3. 139.30.200.74:49154 - Static Voxel Color" << std::endl;
        std::cout << "4. 139.30.200.74:49155 - Dynamic Voxel Color" << std::endl;

        // Create an instance of TopDownViewer
        TopDownViewer Viewer;

        {
            // Lock shared resources before accessing
            std::lock_guard<std::mutex> lockStatic(staticVoxelMutex);
            std::lock_guard<std::mutex> lockDynamic(dynamicVoxelMutex);
            std::lock_guard<std::mutex> lockStaticColor(staticVoxelColorMutex);
            std::lock_guard<std::mutex> lockDynamicColor(dynamicVoxelColorMutex);

            double yaw_rad = staticVoxel_.RPY(2);
            std::vector<Eigen::Vector3f> staticPoints = staticVoxel_.val;
            Eigen::Vector3f vehicle_position = staticVoxel_.NED.cast<float>();
            std::vector<Eigen::Matrix<uint32_t, 3, 1>> static_color = staticVoxelColor_.val;
            float map_res = 0.2;

            std::vector<Eigen::Vector3f> dynamicPoints = dynamicVoxel_.val;
            std::vector<Eigen::Matrix<uint32_t, 3, 1>> dynamic_color = dynamicVoxelColor_.val;

            auto vehicle_mesh = Viewer.CreateVehicleMesh(2.5f, yaw_rad);
            auto static_squares = Viewer.CreateVoxelSquares(staticPoints, vehicle_position, static_color, map_res);
            auto dynamic_squares = Viewer.CreateVoxelSquares(dynamicPoints, vehicle_position, dynamic_color, map_res);

            // Create Open3D visualizer
            open3d::visualization::Visualizer vis;
            vis.CreateVisualizerWindow("Top-Down View", 800, 800);

            // Add point cloud and vehicle mesh to the visualizer
            vis.AddGeometry(vehicle_mesh);
            vis.AddGeometry(static_squares);
            vis.AddGeometry(dynamic_squares);

            // Add NED coordinate frame
            auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0, Eigen::Vector3d(0, 0, 0));
            vis.AddGeometry(coordinate_frame);

            // Run visualization
            vis.Run();

            // Set up a top-down view programmatically
            auto view_control = vis.GetViewControl();

            // Retrieve current camera parameters
            open3d::camera::PinholeCameraParameters camera_params;
            view_control.ConvertToPinholeCameraParameters(camera_params);

            // Modify the camera parameters for a top-down view
            double cameraHeigth = -8;
            camera_params.extrinsic_ <<
                1, 0, 0, 0,  // Camera X-axis
                0, 0, 1, cameraHeigth, // Camera Y-axis (point down -Z and height of 5 units)
                0, -1, 0, 0, // Camera Z-axis (up vector)
                0, 0, 0, 1;  // Homogeneous coordinates

            // Apply the modified parameters back to the view control
            view_control.ConvertFromPinholeCameraParameters(camera_params);

            vis.DestroyVisualizerWindow();
        }

        return 0;

    } else {
        std::cerr << "Listeners failed to start. Exiting program..." << std::endl;
        // Handle the error case (e.g., clean up resources, retry, or exit)
        return EXIT_FAILURE;
    }

    return 0;
}
