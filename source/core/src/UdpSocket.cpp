#include "UdpSocket.hpp"

/**
 * @brief Constructor to initialize the UDP socket.
 * 
 * Initializes the socket, binds it to the specified host and port, 
 * and sets up the callback function.
 */
UDPSocket::UDPSocket(boost::asio::io_context& context, 
                     const std::string& host, 
                     uint16_t port, 
                     std::function<void(const std::vector<uint8_t>&)> callback,
                     uint32_t bufferSize)
    : socket_(context, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(host), port)),
      callback_(callback) {
      buffer_.resize(bufferSize);
      
    startReceive();
}
/**
 * @brief Starts the asynchronous receive operation.
 * 
 * Continuously listens for incoming UDP packets and processes them 
 * using the user-defined callback function.
 */
void UDPSocket::startReceive() {
    socket_.async_receive_from(
        boost::asio::buffer(buffer_), 
        senderEndpoint_,
        [this](boost::system::error_code ec, std::size_t bytesReceived) {
            if (!ec && bytesReceived > 0) {
                // Create a temporary buffer with the exact size of the received packet
                std::vector<uint8_t> packetData(buffer_.begin(), buffer_.begin() + bytesReceived);

                // Invoke the user-defined callback with the received data
                callback_(packetData);
            }
            // Continue receiving data
            startReceive();
        }
    );
}
/**
 * @brief Stops the UDP socket.
 * 
 * Closes the socket and cancels any ongoing asynchronous operations.
 */
void UDPSocket::stop() {
    boost::system::error_code ec;
    socket_.close(ec); // Close the socket
    if (ec) {
        std::cerr << "Error closing socket: " << ec.message() << std::endl;
    }
}
