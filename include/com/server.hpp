#pragma once

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include "control_watchdog.hpp"
#include "uart.hpp"

class Server
{
private:
    static constexpr uint8_t FRAME_HEAD = 0x42;
    static constexpr size_t FRAME_MIN = 4;
    static constexpr size_t FRAME_MAX = 12;

    int socketId{-1};
    std::atomic<int> clientSocket{-1};
    struct sockaddr_in address{};
    std::thread threadRes;
    std::atomic<bool> running{false};
    std::atomic<bool> clientConnected{false};
    std::atomic<bool> connectionValid{false};
    ControlWatchdogState controlWatchdog;
    std::mutex forwardMutex;
    std::vector<uint8_t> rxBuffer;

    static int64_t nowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void stopVehicleLocked()
    {
        uart.carControl(0.0f, 1500);
    }

    void invalidateConnection(int socket)
    {
        connectionValid = false;
        clientConnected = false;
        controlWatchdog.reset();
        if (socket >= 0)
            shutdown(socket, SHUT_RDWR);
        std::lock_guard<std::mutex> lock(forwardMutex);
        stopVehicleLocked();
    }

    void parseFrames()
    {
        while (true)
        {
            auto head = std::find(rxBuffer.begin(), rxBuffer.end(), FRAME_HEAD);
            if (head != rxBuffer.begin())
                rxBuffer.erase(rxBuffer.begin(), head);
            if (rxBuffer.size() < 3)
                return;

            const size_t frameLength = rxBuffer[2];
            if (frameLength < FRAME_MIN || frameLength > FRAME_MAX)
            {
                rxBuffer.erase(rxBuffer.begin());
                continue;
            }
            if (rxBuffer.size() < frameLength)
                return;

            uint8_t check = 0;
            for (size_t i = 0; i + 1 < frameLength; ++i)
                check = static_cast<uint8_t>(check + rxBuffer[i]);
            if (check == rxBuffer[frameLength - 1])
            {
                std::lock_guard<std::mutex> lock(forwardMutex);
                if (connectionValid)
                {
                    uart.transmitFrame(rxBuffer.data(), frameLength);
                    controlWatchdog.onValidFrame(
                        rxBuffer[1], USB_ADDR_CARCTRL, nowMs());
                }
            }
            else
            {
                std::cerr << "[Boot] Dropped control frame with bad checksum\n";
            }
            rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + frameLength);
        }
    }

    void receiveLoop()
    {
        while (running)
        {
            socklen_t addrlen = sizeof(address);
            const int client = accept(socketId, reinterpret_cast<struct sockaddr *>(&address),
                                      &addrlen);
            if (client < 0)
            {
                if (running && errno != EINTR)
                    perror("accept");
                continue;
            }

            clientSocket = client;
            rxBuffer.clear();
            controlWatchdog.onConnected(nowMs());
            connectionValid = true;
            clientConnected = true;
            {
                std::lock_guard<std::mutex> lock(forwardMutex);
                stopVehicleLocked();
            }

            uint8_t buffer[1024];
            while (running && connectionValid)
            {
                const ssize_t length = recv(client, buffer, sizeof(buffer), 0);
                if (length <= 0)
                    break;
                rxBuffer.insert(rxBuffer.end(), buffer, buffer + length);
                parseFrames();
            }

            invalidateConnection(client);
            const int closingClient = clientSocket.exchange(-1);
            if (closingClient >= 0)
                ::close(closingClient);
        }
    }

public:
    Server() = default;
    ~Server() { closeServer(); }

    Uart uart;

    bool start()
    {
        if (running)
            return true;
        if (uart.open("/dev/ttyUSB0") != 0)
        {
            std::cerr << "[Error] Uart Open failed!\n";
            return false;
        }
        uart.startReceive();

        socketId = socket(AF_INET, SOCK_STREAM, 0);
        if (socketId < 0)
        {
            perror("socket failed");
            uart.close();
            return false;
        }
        int reuse = 1;
        setsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(8899);
        if (bind(socketId, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0 ||
            listen(socketId, 3) < 0)
        {
            perror("bind/listen failed");
            ::close(socketId);
            socketId = -1;
            uart.close();
            return false;
        }

        running = true;
        threadRes = std::thread(&Server::receiveLoop, this);
        return true;
    }

    bool isClientConnected() const { return clientConnected.load(); }

    bool watchdogExpired(int64_t timeoutMs) const
    {
        return clientConnected && controlWatchdog.expired(nowMs(), timeoutMs);
    }

    bool watchdogArmed() const { return controlWatchdog.armed(); }

    bool watchdogStartupExpired(int64_t graceMs) const
    {
        return clientConnected && controlWatchdog.startupExpired(nowMs(), graceMs);
    }

    void handleWatchdogTimeout()
    {
        const int activeClient = clientSocket.load();
        invalidateConnection(activeClient);
    }

    void closeServer()
    {
        if (!running.exchange(false))
            return;
        const int activeClient = clientSocket.load();
        invalidateConnection(activeClient);
        if (socketId >= 0)
        {
            const int listeningSocket = socketId;
            socketId = -1;
            shutdown(listeningSocket, SHUT_RDWR);
            ::close(listeningSocket);
        }
        if (threadRes.joinable())
            threadRes.join();
        const int remainingClient = clientSocket.exchange(-1);
        if (remainingClient >= 0)
            ::close(remainingClient);
        uart.close();
    }

    void transmit(const std::string &data)
    {
        const int activeClient = clientSocket.load();
        if (!clientConnected || activeClient < 0)
            return;
        if (send(activeClient, data.data(), data.size(), MSG_NOSIGNAL) < 0)
            perror("send failed");
    }
};
