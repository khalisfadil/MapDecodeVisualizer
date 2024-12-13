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
 * @brief Entry point of the program that initializes and manages listeners, processors, and signal handling.
 *
 * @details 
 * This function performs the following tasks:
 * - Initializes the `vizPointsUtils` object to manage various listeners and processing pipelines.
 * - Registers signal handlers (`SIGINT` and `SIGTERM`) to gracefully shut down the program.
 * - Starts multiple threads to handle:
 *   - UDP-based points and attributes listeners using Boost ASIO.
 *   - Occupancy map pipeline processing at 10 Hz.
 *   - Occupancy map viewer visualization at 10 Hz.
 * - Monitors the `running` flag, which is controlled by the signal handler, to decide when to stop execution.
 * - Ensures all threads are joined, and the program exits gracefully.
 *
 * @return int
 * Returns `EXIT_SUCCESS` if the program exits normally, otherwise `EXIT_FAILURE` on encountering an exception.
 *
 * @throws std::exception
 * Handles any exceptions thrown during the initialization or execution of the listeners or threads.
 */
int main() {

    vizPointsUtils myObject;

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = vizPointsUtils::signalHandler;  
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, nullptr);
    sigaction(SIGTERM, &sigIntHandler, nullptr);

    std::cout << "[Main] Listening to incoming UDP packets..." << std::endl;

     CallbackPoints::Points points;
     CallbackPoints callbackPoints;

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
                myObject.startPointsListener(ioContextPoints,
                                                pointsHost, 
                                                pointsPort, 
                                                1393, 
                                                std::vector<int>{8});
            }
        );

        // // Start attributes Listener using the static method
        // boost::asio::io_context ioContextAttributes;
        // threads.emplace_back(
        //     [&]() { 
        //         myObject.startAttributesListener(ioContextAttributes,
        //                                             attributesHost, 
        //                                             attributesPort, 
        //                                             1393, 
        //                                             std::vector<int>{9});
        //     }
        // );

        // // Start Processing (10 Hz)
        // threads.emplace_back([&]() {
        //     myObject.runOccupancyMapPipeline(std::vector<int>{0, 1, 2, 3});
        //     }
        // );

        // // Start Processing (10 Hz)
        // threads.emplace_back([&]() {
        //     myObject.runOccupancyMapViewer(std::vector<int>{4, 5, 6, 7});
        //     }
        // );

        // Monitor signal and clean up
        while (vizPointsUtils::running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop IO contexts
        ioContextPoints.stop();
        //ioContextAttributes.stop();

        // Join all threads
        for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: [Main] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "[Main]All listeners stopped. Exiting program." << std::endl;
    return EXIT_SUCCESS;
}