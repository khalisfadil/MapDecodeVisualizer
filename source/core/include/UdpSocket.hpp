#pragma once

#include <cstdint>
#include <iostream>
#include <vector>
#include <functional>
#include <boost/asio.hpp>

/**
 * @class UDPSocket
 * @brief A class for handling UDP communication using Boost.Asio.
 * 
 * This class provides an interface for creating and managing UDP sockets,
 * and processing incoming data through a user-defined callback function.
 */
class UDPSocket {
public:
    /**
     * @brief Constructor to initialize the UDP socket.
     * 
     * @param context Reference to the Boost.Asio IO context.
     * @param host The host address to bind the socket to.
     * @param port The port number to bind the socket to.
     * @param callback A user-defined callback function for processing incoming data.
     * @param bufferSize The maximum size of the buffer for incoming packets.
     */
    UDPSocket(boost::asio::io_context& context, 
              const std::string& host, 
              uint16_t port, 
              std::function<void(const std::vector<uint8_t>&)> callback,
              uint32_t bufferSize = 65535);

    /**
     * @brief Starts the asynchronous receive operation.
     * 
     * This method initializes the asynchronous receive loop for incoming UDP packets.
     */
    void startReceive();

    /**
     * @brief Stops the UDP socket.
     * 
     * Closes the socket and cancels any ongoing operations.
     */
    void stop();

private:
    boost::asio::ip::udp::socket socket_;                         ///< UDP socket for communication.
    boost::asio::ip::udp::endpoint senderEndpoint_;               ///< Endpoint of the sender.
    std::vector<uint8_t> buffer_;                                 ///< Buffer for receiving incoming packets.
    std::function<void(const std::vector<uint8_t>&)> callback_;   ///< User-defined callback function.
};

