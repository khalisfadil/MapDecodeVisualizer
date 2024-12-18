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

#include <cstdint>
#include <iostream>
#include <vector>
#include <functional>
#include <boost/asio.hpp>
#include <iomanip>

// -----------------------------------------------------------------------------
/**
 * @class UDPSocket
 * @brief A class for handling UDP communication using Boost.Asio.
 *
 * This class provides an interface for creating and managing UDP sockets,
 * receiving incoming data, and processing it through a user-defined callback function.
 */
class UDPSocket {
public:

    // -----------------------------------------------------------------------------
    /**
     * @brief Constructor to initialize the UDP socket.
     * 
     * @param context Reference to the Boost.Asio IO context, required for asynchronous operations.
     * @param host The host address to bind the socket to (e.g., "127.0.0.1").
     * @param port The port number to bind the socket to.
     * @param callback A user-defined callback function for processing incoming data.
     *                 The callback is called with a vector containing the received packet's data.
     * @param bufferSize The maximum size of the buffer for incoming packets. Default is 65535 bytes.
     */
    UDPSocket(boost::asio::io_context& context, 
              const std::string& host, 
              uint16_t port, 
              std::function<void(const std::vector<uint8_t>&)> callback,
              uint32_t bufferSize = 65535);

    // -----------------------------------------------------------------------------
    /**
     * @brief Destructor to clean up the UDP socket.
     *
     * Ensures the socket is properly closed and any ongoing operations are canceled.
     */
    ~UDPSocket();

    // -----------------------------------------------------------------------------
    /**
     * @brief Starts the asynchronous receive operation.
     *
     * This method sets up an asynchronous receive loop for incoming UDP packets. 
     * Incoming data is passed to the user-defined callback function.
     */
    void startReceive();

    // -----------------------------------------------------------------------------
    /**
     * @brief Stops the UDP socket.
     *
     * Closes the socket and cancels any ongoing operations. Should be called to cleanly
     * shut down the socket before destruction.
     */
    void stop();

private:

    // -----------------------------------------------------------------------------
    /**
     * @brief UDP socket used for communication.
     *
     * This socket is responsible for sending and receiving UDP packets.
     * It is initialized with the appropriate I/O context and endpoint for communication.
     */
    boost::asio::ip::udp::socket socket_;                         

    // -----------------------------------------------------------------------------
    /**
     * @brief Endpoint of the sender.
     *
     * This variable holds information about the remote sender's address and port.
     * It is updated with the sender's details whenever a new packet is received.
     */
    boost::asio::ip::udp::endpoint senderEndpoint_;               

    // -----------------------------------------------------------------------------
    /**
     * @brief Buffer for storing incoming packet data.
     *
     * This vector serves as temporary storage for raw data received over the UDP socket.
     * It is passed to the callback function for processing after data is received.
     */
    std::vector<uint8_t> buffer_;                                 

    // -----------------------------------------------------------------------------
    /**
     * @brief Callback for processing incoming packets.
     *
     * This callback function is invoked whenever a new packet is received.
     * It processes the data stored in the buffer and enables custom handling
     * of the incoming packet by the application.
     *
     * @param data The raw packet data received from the UDP socket.
     */
    std::function<void(const std::vector<uint8_t>&)> callback_; 
      
};

