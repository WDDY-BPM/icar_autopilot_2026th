#pragma once

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include "uart.hpp"

class Server
{
private:
    static constexpr uint8_t FRAME_HEAD = 0x42;
    static constexpr size_t FRAME_MIN = 4;
    static constexpr size_t FRAME_MAX = 12;

    int socketId{-1};
    std::atomic<int> newSocket{-1};
    struct sockaddr_in address{};
    std::thread threadRes;
    std::atomic<bool> running{false};
    std::vector<uint8_t> rxBuffer;

    void stopVehicle()
    {
        uart.carControl(0.0f, 1500);
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
                for (size_t i = 0; i < frameLength; ++i)
                    uart.transmitByte(rxBuffer[i]);
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
            newSocket.store(client);

            rxBuffer.clear();
            startApp = true;
            countDrop = 0;
            uint8_t buffer[1024];
            while (running)
            {
                const ssize_t len = recv(client, buffer, sizeof(buffer), 0);
                if (len <= 0)
                    break;
                startApp = true;
                countDrop = 0;
                rxBuffer.insert(rxBuffer.end(), buffer, buffer + len);
                parseFrames();
            }

            stopVehicle();
            startApp = false;
            const int closingClient = newSocket.exchange(-1);
            if (closingClient >= 0)
            {
                shutdown(closingClient, SHUT_RDWR);
                ::close(closingClient);
            }
        }
    }

public:
    Server() = default;
    ~Server() { closeServer(); }

    Uart uart;
    std::atomic<int> countDrop{0};
    std::atomic<bool> startApp{false};

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

    void closeServer()
    {
        if (!running.exchange(false))
            return;
        stopVehicle();
        const int activeClient = newSocket.load();
        if (activeClient >= 0)
            shutdown(activeClient, SHUT_RDWR);
        if (socketId >= 0)
        {
            int listeningSocket = socketId;
            socketId = -1;
            shutdown(listeningSocket, SHUT_RDWR);
            ::close(listeningSocket); // unblock accept before join
        }
        if (threadRes.joinable())
            threadRes.join();
        const int remainingClient = newSocket.exchange(-1);
        if (remainingClient >= 0)
            ::close(remainingClient);
        uart.close();
    }

    void handleWatchdogTimeout()
    {
        stopVehicle();
        startApp = false;
        countDrop = 0;
        const int activeClient = newSocket.load();
        if (activeClient >= 0)
            shutdown(activeClient, SHUT_RDWR);
    }

    void transmit(const std::string &data)
    {
        const int activeClient = newSocket.load();
        if (!startApp || activeClient < 0)
            return;
        if (send(activeClient, data.data(), data.size(), MSG_NOSIGNAL) < 0)
            perror("send failed");
    }
};
