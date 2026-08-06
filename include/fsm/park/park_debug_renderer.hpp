#pragma once
#include <chrono>
#include <opencv2/core.hpp>

class ParkDebugRenderer
{
public:
    void maybeSave(bool debug, bool saveImages, cv::Mat &image,
                   cv::Mat &ipmImage);
private:
    std::chrono::steady_clock::time_point lastSavedAt{};
};
