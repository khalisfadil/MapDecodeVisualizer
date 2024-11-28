#pragma once

#include <cstdint>
#include <iostream>
#include <vector>
#include <functional>
#include <boost/asio.hpp>
#include <iomanip>

/**
 * @class UDPSocket
 * @brief A class for handling UDP communication using Boost.Asio.
 *
 * This class provides an interface for creating and managing UDP sockets,
 * receiving incoming data, and processing it through a user-defined callback function.
 */
class UDPSocket {
public:
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

    /**
     * @brief Destructor to clean up the UDP socket.
     *
     * Ensures the socket is properly closed and any ongoing operations are canceled.
     */
    ~UDPSocket();

    /**
     * @brief Starts the asynchronous receive operation.
     *
     * This method sets up an asynchronous receive loop for incoming UDP packets. 
     * Incoming data is passed to the user-defined callback function.
     */
    void startReceive();

    /**
     * @brief Stops the UDP socket.
     *
     * Closes the socket and cancels any ongoing operations. Should be called to cleanly
     * shut down the socket before destruction.
     */
    void stop();

private:
    boost::asio::ip::udp::socket socket_;                         ///< UDP socket used for communication.
    boost::asio::ip::udp::endpoint senderEndpoint_;               ///< Endpoint of the sender.
    std::vector<uint8_t> buffer_;                                 ///< Buffer for storing incoming packet data.
    std::function<void(const std::vector<uint8_t>&)> callback_;   ///< Callback for processing incoming packets.
};

