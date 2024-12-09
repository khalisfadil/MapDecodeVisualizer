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
#include "udpSocket.hpp"

// -----------------------------------------------------------------------------
// Section: UDPSocket
// -----------------------------------------------------------------------------

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


// -----------------------------------------------------------------------------
// Section: UDPSocket
// -----------------------------------------------------------------------------

UDPSocket::~UDPSocket() {
    stop();
}

// -----------------------------------------------------------------------------
// Section: startReceive
// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// Section: stop
// -----------------------------------------------------------------------------

void UDPSocket::stop() {
    boost::system::error_code ec;
    socket_.close(ec); // Close the socket
    if (ec) {
        std::cerr << "Error closing socket: " << ec.message() << std::endl;
    }
}
