#include "udpSocket.hpp"
/**
 * @brief Constructor to initialize the UDP socket.
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
 * @brief Destructor to ensure socket cleanup.
 */
UDPSocket::~UDPSocket() {
    stop();
}

/**
 * @brief Starts the asynchronous receive operation.
 */
void UDPSocket::startReceive() {
    socket_.async_receive_from(
        boost::asio::buffer(buffer_), 
        senderEndpoint_,
        [this](boost::system::error_code ec, std::size_t bytesReceived) {
            if (!ec && bytesReceived > 0) {
                std::vector<uint8_t> packetData(buffer_.begin(), buffer_.begin() + bytesReceived);

                #ifdef DEBUG
                std::cout << "Received packet (size: " << bytesReceived << "): ";
                for (size_t i = 0; i < bytesReceived; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') 
                              << static_cast<int>(packetData[i]) << " ";
                }
                std::cout << std::dec << std::endl;
                #endif

                if (callback_) {
                    callback_(packetData);
                }
            } else if (ec) {
                std::cerr << "Receive error: " << ec.message() << std::endl;
            }

            startReceive();
        }
    );
}

/**
 * @brief Stops the UDP socket.
 */
void UDPSocket::stop() {
    boost::system::error_code ec;
    socket_.close(ec);
    if (ec) {
        std::cerr << "Error closing socket: " << ec.message() << std::endl;
    }
}
