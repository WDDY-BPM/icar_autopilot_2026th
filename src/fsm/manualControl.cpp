/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstukeji.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file manualControl.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 手动接管控制线程实现
 * @version 0.1
 * @date 2025-12-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/manualControl.hpp"
#include "utils/tools.hpp"
#include <unistd.h>
#include <signal.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <algorithm>

namespace {
bool sendAll(int socketFd, const void *data, size_t length)
{
    const char *cursor = static_cast<const char *>(data);
    while (length > 0)
    {
        ssize_t sent = send(socketFd, cursor, length, MSG_NOSIGNAL);
        if (sent < 0 && errno == EINTR)
            continue;
        if (sent <= 0)
            return false;
        cursor += sent;
        length -= static_cast<size_t>(sent);
    }
    return true;
}
}

ManualControlThread::ManualControlThread() {
    serverSocket = -1;
    clientSocket = -1;
    running = false;
    connected = false;
    vehicleState.speed = 0.0f;
    vehicleState.steering = PWMSERVOMID;
    vehicleState.emergency = false;
    vehicleState.manual = false;
    const char *token = std::getenv("ICAR_MANUAL_TOKEN");
    const char *source = std::getenv("ICAR_MANUAL_ALLOWED_IP");
    authToken = token ? token : "";
    allowedIp = source ? source : "";
}

ManualControlThread::~ManualControlThread() {
    stop();
}

void ManualControlThread::start() {
    signal(SIGPIPE, SIG_IGN);
    if (authToken.empty()) {
        std::cerr << "[Manual] ICAR_MANUAL_TOKEN is not set; takeover service disabled\n";
        return;
    }
    running = true;
    lastContact = std::chrono::steady_clock::now();

    // Create server socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0)
    {
        perror("[Manual] socket failed");
        running = false;
        return;
    }

    // 允许端口立即重用，防止程序重启后TIME_WAIT导致bind失败
    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        perror("[Manual] setsockopt SO_REUSEADDR failed");
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(serverSocket.load(), (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        perror("[Manual] bind failed");
        close(serverSocket);
        serverSocket = -1;
        running = false;
        return;
    }

    if (listen(serverSocket, 1) < 0)
    {
        perror("[Manual] listen failed");
        close(serverSocket);
        serverSocket = -1;
        running = false;
        return;
    }

    thread = std::thread(&ManualControlThread::run, this);
}

void ManualControlThread::stop() {
    running = false;
    connected = false;

    const int activeClient = clientSocket.load();
    if (activeClient >= 0)
        shutdown(activeClient, SHUT_RDWR);
    const int listeningSocket = serverSocket.load();
    if (listeningSocket >= 0)
        shutdown(listeningSocket, SHUT_RDWR);

    if (thread.joinable())
        thread.join();

    const int remainingClient = clientSocket.exchange(-1);
    if (remainingClient >= 0)
        close(remainingClient);
    const int remainingServer = serverSocket.exchange(-1);
    if (remainingServer >= 0)
        close(remainingServer);
}
void ManualControlThread::sendImage(cv::Mat &img) {
    if (!img.empty()) {
        std::lock_guard<std::mutex> lock(mtxImg);
        image = img.clone();
        hasImage = true;
        cvImg.notify_one();
    }
}

void ManualControlThread::updateVehicleState(float speed, float steering, bool manual) {
    std::lock_guard<std::mutex> lock(mtxState);
    vehicleState.speed = speed;
    vehicleState.steering = steering;
    vehicleState.emergency = manualControl.emergencyStopRequested.load();
    vehicleState.manual = manual;
}

bool ManualControlThread::isManualControl() {
    return manualControl.forward || manualControl.backward ||
           manualControl.left || manualControl.right;
}

void ManualControlThread::applyManualControl(float *speed, uint16_t *steering) {

    if (manualControl.emergencyStop) {
        *speed = 0;
        *steering = PWMSERVOMID; // 中间位置
        return;
    }

    // Speed control
    if (manualControl.forward) {
        *speed = 0.3f;  // Forward speed
    } else if (manualControl.backward) {
        *speed = -0.3f; // Backward speed
    } else {
        *speed = 0.0f;  // Stop
    }

    // Steering control
    if (manualControl.left) {
        *steering = PWMSERVOMID - 300; // 左转
    } else if (manualControl.right) {
        *steering = PWMSERVOMID + 300; // 右转
    } else {
        *steering = PWMSERVOMID;  // 直行
    }
}

bool ManualControlThread::checkForReturnKey() {
    return manualControl.returnAuto.exchange(false);
}

void ManualControlThread::disconnectClient() {
    connected = false;
    const int socket = clientSocket.load();
    if (socket >= 0)
        shutdown(socket, SHUT_RDWR);
}

bool ManualControlThread::isEmergencyStopRequested() const {
    return manualControl.emergencyStopRequested.load();
}
void ManualControlThread::run() {
    while (running) {
        // 如果socket未成功初始化（start失败），等待重试
        if (serverSocket < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Wait for client connection
        addrSize = sizeof(clientAddr);
        clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &addrSize);

        if (clientSocket < 0) {
            continue;
        }

        if (!authenticateClient()) {
            close(clientSocket);
            clientSocket = -1;
            continue;
        }

        resetManualControl(true);
        connected = true;
        lastContact = std::chrono::steady_clock::now();  // 重置超时计时（防止旧连接的lastContact导致立即超时）
        // 禁用Nagle算法，降低控制延迟
        int flag = 1;
        setsockopt(clientSocket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        printf("[Manual] Remote control connected\n");

        // Handle client connection
        handleClientConnection();

        connected = false;
        manualControl.forward = false;
        manualControl.backward = false;
        manualControl.left = false;
        manualControl.right = false;
        manualControl.emergencyStop = true;
        if (clientSocket >= 0) {
            close(clientSocket);
            clientSocket = -1;
        }
    }
}

bool ManualControlThread::authenticateClient() {
    char peer[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &clientAddr.sin_addr, peer, sizeof(peer));
    if (!allowedIp.empty() && allowedIp != peer) {
        std::cerr << "[Manual] Rejected source " << peer << "\n";
        return false;
    }

    timeval timeout{3, 0};
    setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    std::string line;
    bool complete = false;
    char byte = 0;
    while (line.size() <= 512 && recv(clientSocket, &byte, 1, 0) == 1) {
        if (byte == '\n') {
            complete = true;
            break;
        }
        if (byte != '\r')
            line.push_back(byte);
    }
    timeout = {0, 0};
    setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    const std::string supplied = complete && line.rfind("AUTH:", 0) == 0 ? line.substr(5) : "";
    unsigned char difference = static_cast<unsigned char>(supplied.size() ^ authToken.size());
    const size_t compared = std::max(supplied.size(), authToken.size());
    for (size_t i = 0; i < compared; ++i) {
        const unsigned char lhs = i < supplied.size() ? supplied[i] : 0;
        const unsigned char rhs = i < authToken.size() ? authToken[i] : 0;
        difference |= lhs ^ rhs;
    }
    if (difference != 0) {
        std::cerr << "[Manual] Authentication failed for " << peer << "\n";
        return false;
    }
    return true;
}

void ManualControlThread::resetManualControl(bool emergency, bool preserveReturnAuto) {
    manualControl.forward = false;
    manualControl.backward = false;
    manualControl.left = false;
    manualControl.right = false;
    manualControl.emergencyStop = emergency;
    if (!preserveReturnAuto)
        manualControl.returnAuto = false;
    controlChanged = true;
}
void ManualControlThread::handleClientConnection() {

    resetManualControl(false);


    // Start receiving commands thread
    std::thread cmdThread(&ManualControlThread::receiveCommands, this);

    // Start timeout check thread
    std::thread timeoutThread([this]() {
        while (running && connected) {
            checkTimeout();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    // Send images and state
    while (running && connected) {
        // Send vehicle state (higher frequency for state)
        std::string state;
        {
            std::lock_guard<std::mutex> lock(mtxState);
            state = "STATE:" + std::to_string(vehicleState.speed) +
                    "," + std::to_string(vehicleState.steering) +
                    "," + (vehicleState.manual ? "MANUAL" : "AUTO") +
                    "," + (vehicleState.emergency ? "ESTOP" : "NORMAL") + "\n";
        }

        // 发送状态数据（带错误检查）
        if (!sendAll(clientSocket, state.data(), state.size())) {
            cerr << "[Manual] Error sending state data" << endl;
            break;
        }

        // Copy the newest frame under the lock, then encode/send without
        // blocking the vehicle control thread.
        cv::Mat frameToSend;
        if (hasImage) {
            std::lock_guard<std::mutex> imgLock(mtxImg);
            if (!image.empty())
                frameToSend = image.clone();
            hasImage = false;
        }
        if (!frameToSend.empty()) {
            try {
                std::vector<uchar> buf;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
                cv::imencode(".jpg", frameToSend, buf, params);
                std::string header = "IMAGE:" + std::to_string(buf.size()) + "\n";
                if (!sendAll(clientSocket, header.data(), header.size()) ||
                    !sendAll(clientSocket, buf.data(), buf.size()))
                {
                    connected = false;
                    break;
                }
            } catch (const cv::Exception& e) {
                cerr << "[Manual] Image encode error: " << e.what() << endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33));  // 约30 FPS
    }
    connected = false;
    if (clientSocket >= 0)
        shutdown(clientSocket, SHUT_RDWR);
    if (cmdThread.joinable())
        cmdThread.join();
    if (timeoutThread.joinable())
        timeoutThread.join();
}

void ManualControlThread::receiveCommands() {
    std::string lineBuf;  // 粘包缓冲区
    char rawBuf[1024];
    while (running && connected) {
        int bytes = recv(clientSocket, rawBuf, 1023, 0);
        if (bytes <= 0) {
            break;
        }

        rawBuf[bytes] = '\0';
        lineBuf += std::string(rawBuf);

        // Update contact time
        {
            std::lock_guard<std::mutex> contactLock(mtxContact);
            lastContact = std::chrono::steady_clock::now();
        }

        // 按换行符分割处理（解决TCP粘包）
        size_t pos;
        while ((pos = lineBuf.find('\n')) != std::string::npos) {
            std::string cmd = lineBuf.substr(0, pos + 1);  // 包含\n
            lineBuf.erase(0, pos + 1);

            processCommand(cmd);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
done:
    lineBuf.clear();
}

void ManualControlThread::processCommand(const std::string &cmd) {
    bool manualMode = false;
    {
        std::lock_guard<std::mutex> lock(mtxState);
        manualMode = vehicleState.manual;
    }
    if (cmd != "PING\n" && cmd != "STOP\n" && cmd != "CLEAR_STOP\n" && !manualMode) {
        resetManualControl(true);
        std::cerr << "[Manual] Ignored motion command while vehicle is in AUTO\n";
        return;
    }
    // 处理特殊命令
    if (cmd == "RETURN\n") {
        manualControl.returnAuto = true;
        controlChanged = true;
        printf("[Manual] Return to auto command received\n");
        return;
    }
    if (cmd == "PING\n") {
        // Heartbeat/dead-man refresh only. Never consume the latched RETURN
        // request or alter emergency-stop state.
        manualControl.forward = false;
        manualControl.backward = false;
        manualControl.left = false;
        manualControl.right = false;
        controlChanged = true;
        return;
    }
    if (cmd == "CLEAR_STOP\n") {
        manualControl.emergencyStopRequested = false;
        resetManualControl(false, true); // RETURN remains latched until main loop consumes it
        printf("[Manual] Latched emergency stop cleared\n");
        return;
    }
    if (cmd == "STOP\n") {
        manualControl.forward = false;
        manualControl.backward = false;
        manualControl.left = false;
        manualControl.right = false;
        manualControl.emergencyStop = true;
        manualControl.emergencyStopRequested = true;
        controlChanged = true;
        printf("[Manual] Stop command received\n");
        return;
    }

    // 组合命令解析（如"WA\n"=前进+左转，"WD\n"=前进+右转）
    manualControl.forward = false;
    manualControl.backward = false;
    manualControl.left = false;
    manualControl.right = false;
    manualControl.emergencyStop = false;
    for (char c : cmd) {
        switch (c) {
            case 'W': manualControl.forward = true; break;
            case 'S': manualControl.backward = true; break;
            case 'A': manualControl.left = true; break;
            case 'D': manualControl.right = true; break;
        }
    }
    controlChanged = true;
    if (cmd != "PING\n")
        printf("[Manual] Command received: %s", cmd.c_str());
}

void ManualControlThread::checkTimeout() {
    if (connected) {
        auto now = std::chrono::steady_clock::now();
        long long elapsed;
        {
            std::lock_guard<std::mutex> contactLock(mtxContact);
            elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastContact).count();
        }

        if (elapsed > 500) { // 500 ms dead-man timeout
            printf("[Manual] Connection timeout, stopping vehicle\n");
            manualControl.forward = false;
            manualControl.backward = false;
            manualControl.left = false;
            manualControl.right = false;
            manualControl.emergencyStop = true;
            emergencyStop();
            connected = false;
            if (clientSocket >= 0)
                shutdown(clientSocket, SHUT_RDWR);
        }
    }
}

void ManualControlThread::emergencyStop() {
    std::lock_guard<std::mutex> lock(mtxState);
    vehicleState.speed = 0;
    vehicleState.steering = PWMSERVOMID;
    vehicleState.emergency = true;
}
