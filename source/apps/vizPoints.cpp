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
/**
 * @brief Main entry point for the program that listens to incoming UDP packets and processes them.
 *
 * @detail This function initializes the `vizPointsUtils` class, registers signal handlers for termination signals (SIGINT and SIGTERM), and starts multiple threads to handle UDP listening for points and attributes. It also manages the synchronization of data between threads using condition variables and mutexes. The program listens for incoming data, processes it in real-time (at a frequency of 10 Hz), and gracefully shuts down when a termination signal is received.
 *
 * The flow of execution includes:
 * - Registering signal handlers for graceful shutdown.
 * - Starting listener threads for receiving points and attributes data.
 * - Starting a processing thread to handle the data at regular intervals.
 * - Monitoring the program's running state and stopping the listeners and threads when the program is signaled to terminate.
 */
int main() {

    // Register signal handler using the static signalHandler method
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = vizPointsUtils::signalHandler;  // Directly use static method
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, nullptr);
    sigaction(SIGTERM, &sigIntHandler, nullptr);

    std::cout << "Listening to incoming UDP packets..." << std::endl;

    // Shared resources
    CallbackPoints::Points points_, attributes_;
    CallbackPoints callbackPoints_, callbackAttributes_;

    try {
        std::vector<std::thread> threads;

        // Listener configurations
        std::string pointsHost = "127.0.0.1";
        uint16_t pointsPort = 61234;
        std::string attributesHost = "127.0.0.1";
        uint16_t attributesPort = 61235;

        // Start points Listener using the static method
        boost::asio::io_context ioContextPoints;
        threads.emplace_back(
            [&]() { 
                vizPointsUtils::startListener(
                    ioContextPoints, 
                    pointsHost, 
                    pointsPort, 
                    1393, 
                    std::vector<int>{8}, 
                    callbackPoints_, 
                    points_,
                    vizPointsUtils::consoleMutex, 
                    vizPointsUtils::pointsMutex,  
                    vizPointsUtils::pointsDataReadyCV, 
                    vizPointsUtils::pointsDataAvailable,
                    vizPointsUtils::running
                );
            }
        );

        // Start attributes Listener using the static method
        boost::asio::io_context ioContextAttributes;
        threads.emplace_back(
            [&]() { 
                vizPointsUtils::startListener(
                    ioContextAttributes, 
                    attributesHost, 
                    attributesPort, 
                    1393, 
                    std::vector<int>{9}, 
                    callbackAttributes_, 
                    attributes_,
                    vizPointsUtils::consoleMutex, 
                    vizPointsUtils::attributesMutex,  // Access static mutex
                    vizPointsUtils::attributesDataReadyCV, 
                    vizPointsUtils::attributesDataAvailable,
                    vizPointsUtils::running
                );
            }
        );

        // Start Processing (10 Hz)
        threads.emplace_back([&]() {
            vizPointsUtils::runOccupancyMapPipeline(points_, 
                                                    attributes_,
                                                    vizPointsUtils::frameID,
                                                    vizPointsUtils::position,
                                                    vizPointsUtils::orientation,
                                                    vizPointsUtils::receivedStaticVoxels,
                                                    vizPointsUtils::receivedOccupancyColors,
                                                    vizPointsUtils::receivedReflectivityColors,
                                                    vizPointsUtils::receivedIntensityColors,
                                                    vizPointsUtils::receivedNIRColors,
                                                    std::vector<int>{0, 1, 2, 3},
                                                    vizPointsUtils::running,
                                                    vizPointsUtils::consoleMutex,
                                                    vizPointsUtils::pointsMutex,
                                                    vizPointsUtils::attributesMutex);
        });

        // // Start Processing (10 Hz)
        // threads.emplace_back([&]() {
        //     vizPointsUtils::runOccupancyMapViewer(vizPointsUtils::frameID, 
        //                                             vizPointsUtils::position,
        //                                             vizPointsUtils::orientation,
        //                                             vizPointsUtils::receivedStaticVoxels,
        //                                             vizPointsUtils::receivedOccupancyColors,
        //                                             vizPointsUtils::receivedReflectivityColors,
        //                                             vizPointsUtils::receivedIntensityColors,
        //                                             vizPointsUtils::receivedNIRColors,
        //                                             std::vector<int>{4, 5, 6, 7},
        //                                             vizPointsUtils::running,
        //                                             vizPointsUtils::consoleMutex,
        //                                             vizPointsUtils::pointsMutex,
        //                                             vizPointsUtils::attributesMutex);
        // });

        // Monitor signal and clean up
        while (vizPointsUtils::running) {
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