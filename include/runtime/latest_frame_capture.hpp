#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

class LatestFrameCapture
{
public:
    LatestFrameCapture() = default;
    ~LatestFrameCapture();

    bool open(const std::string &source, int backend = cv::CAP_ANY);
    bool isOpened() const;
    bool set(int property, double value);
    double get(int property) const;
    bool read(cv::Mat &frame);
    void start();
    bool waitForLatest(cv::Mat &frame, std::chrono::milliseconds timeout);
    void stop();

private:
    void captureLoop();

    std::shared_ptr<cv::VideoCapture> capture_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    mutable std::mutex frameMutex_;
    std::condition_variable frameReady_;
    cv::Mat latestFrame_;
    std::uint64_t latestSequence_{0};
    std::uint64_t consumedSequence_{0};
};
