#include "udpSocket.hpp"

/**
 * @brief Constructor to initialize the UDP socket.
 * 
 * Sets up the UDP socket with the provided host and port, initializes the receive buffer,
 * and starts listening for incoming packets.
 * 
 * @param context Reference to the Boost.Asio IO context.
 * @param host The host address to bind the socket to (e.g., "127.0.0.1").
 * @param port The port number to bind the socket to.
 * @param callback A user-defined callback function to handle received data.
 * @param bufferSize The size of the buffer for incoming packets (default is 65535).
 */
UDPSocket::UDPSocket(boost::asio::io_context& context, 
                     const std::string& host, 
                     uint16_t port, 
                     std::function<void(const std::vector<uint8_t>&)> callback,
                     uint32_t bufferSize)
    : socket_(context, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(host), port)),
      callback_(std::move(callback)) {  // Use std::move to avoid unnecessary copying of the callback
    buffer_.resize(bufferSize);         // Allocate memory for the buffer
    startReceive();                     // Start listening for incoming packets
}

/**
 * @brief Destructor to ensure socket cleanup.
 * 
 * Calls the stop method to close the socket and cancel any ongoing operations.
 */
UDPSocket::~UDPSocket() {
    stop();
}

/**
 * @brief Starts the asynchronous receive operation.
 * 
 * This method sets up an asynchronous receive loop to handle incoming UDP packets.
 * Incoming data is passed to the user-defined callback function.
 */
void UDPSocket::startReceive() {
    socket_.async_receive_from(
        boost::asio::buffer(buffer_),    // Buffer for receiving data
        senderEndpoint_,                // Endpoint to store sender's information
        [this](boost::system::error_code ec, std::size_t bytesReceived) {
            if (!ec && bytesReceived > 0) {
                // Copy received data into a vector
                std::vector<uint8_t> packetData(buffer_.begin(), buffer_.begin() + bytesReceived);

                #ifdef DEBUG
                // Print received packet in hex format (for debugging purposes)
                std::cout << "Received packet (size: " << bytesReceived << "): ";
                for (size_t i = 0; i < bytesReceived; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') 
                              << static_cast<int>(packetData[i]) << " ";
                }
                std::cout << std::dec << std::endl;
                #endif

                // Invoke the user-defined callback with the received packet data
                if (callback_) {
                    callback_(packetData);
                }
            } else if (ec) {
                // Log an error if the receive operation failed
                std::cerr << "Receive error: " << ec.message() << std::endl;
            }

            // Continue listening for the next packet
            startReceive();
        }
    );
}

/**
 * @brief Stops the UDP socket.
 * 
 * Closes the socket and cancels any ongoing operations. Logs an error message if the
 * socket fails to close cleanly.
 */
void UDPSocket::stop() {
    boost::system::error_code ec;
    socket_.close(ec); // Close the socket
    if (ec) {
        std::cerr << "Error closing socket: " << ec.message() << std::endl;
    }
}
