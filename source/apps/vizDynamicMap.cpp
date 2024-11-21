#include "udpSocket.hpp"
#include "callbackStaticVoxel.hpp"
#include <iostream>
#include <boost/asio.hpp>

int main() {
    try {
        // Create a Boost.Asio I/O context
        boost::asio::io_context ioContext;

        // Initialize the staticVoxel object
        CallbackStaticVoxel::staticVoxel staticVoxel_;

        // Create the callback object
        CallbackStaticVoxel CallbackStaticVoxel_;

        // Lambda function to handle incoming data
        auto udpCallbackStaticVoxel = [&CallbackStaticVoxel_, &staticVoxel_](const std::vector<uint8_t>& data) {
            // Process the received data using the callbackVoxel
            CallbackStaticVoxel_.process(data, staticVoxel_);

            // // Print the updated staticVoxel data
            // std::cout << "Received Frame ID: " << staticVoxel_.frameID << std::endl;
            // std::cout << "Number of Points: " << staticVoxel_.numVal << std::endl;
            // std::cout << "Timestamp: " << staticVoxel_.t << std::endl;
            // std::cout << "Position (NED): " << staticVoxel_.NED.transpose() << std::endl;
            // std::cout << "Orientation (RPY): " << staticVoxel_.RPY.transpose() << std::endl;

            // // Print the points
            // for (uint32_t i = 0; i < staticVoxel_.numVal; ++i) {
            //     std::cout << "Point " << i + 1 << ": " 
            //               << staticVoxel_.val[i].transpose() << std::endl;
            // }
        };

        // Create a UDP socket, binding to host and port
        const std::string host = "127.0.0.1"; // Listen on localhost
        const uint16_t port = 12345;         // Listening port
        const uint32_t bufferSize = 2048;   // Buffer size for incoming packets

        // Initialize the UDP socket with the new name
        UDPSocket voxelDataListener(ioContext, host, port, udpCallbackStaticVoxel, bufferSize);

        // Run the Boost.Asio I/O context to start listening
        std::cout << "Listening for UDP packets on " << host << ":" << port << "..." << std::endl;
        ioContext.run();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}

// int main() {
//     // Example input points in NED
//     std::vector<Eigen::Vector3f> points = {
//         {10.0f, 10.0f, 0.0f},
//         {12.0f, 10.0f, 0.0f},
//         {10.0f, 12.0f, 0.0f},
//         {12.0f, 12.0f, 0.0f}
//     };

//     // Example grayscale values for points
//     std::vector<uint8_t> grayscale_values = {100, 150, 200, 255};

//     // Vehicle position and orientation in NED
//     Eigen::Vector3f vehicle_position(10.0f, 10.0f, 0.0f); // Vehicle at (10, 10)
//     float yaw = 45.0f; // Vehicle facing 45 degrees

//     // Map resolution
//     float map_resolution = 1.0f;

//     // Create an instance of TopDownViewer
//     TopDownViewer viewer;

//     // Translate points to make the vehicle the center
//     auto translated_points = viewer.TranslatePointsToVehicleCenter(points, vehicle_position);

//     // Create vehicle mesh (triangle aligned with heading)
//     auto vehicle_mesh = viewer.CreateVehicleMesh(1.0f, yaw);

//     // Create Open3D point cloud
//     auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
//     for (size_t i = 0; i < translated_points.size(); ++i) {
//         const auto& point = translated_points[i];
//         point_cloud->points_.emplace_back(point.x(), point.y(), point.z());

//         // Assign grayscale color
//         float color = grayscale_values[i] / 255.0f;
//         point_cloud->colors_.emplace_back(color, color, color); // Grayscale color
//     }

//     // Create Open3D visualizer
//     open3d::visualization::Visualizer vis;
//     vis.CreateVisualizerWindow("Top-Down View", 800, 800);

//     // Add point cloud and vehicle mesh to the visualizer
//     vis.AddGeometry(point_cloud);
//     vis.AddGeometry(vehicle_mesh);

//     // Add NED coordinate frame
//     auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0, Eigen::Vector3d(0, 0, 0));
//     vis.AddGeometry(coordinate_frame);

//     // Run visualization
//     vis.Run();
//     vis.DestroyVisualizerWindow();

//     return 0;
// }

