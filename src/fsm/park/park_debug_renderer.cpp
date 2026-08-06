#include "fsm/park/park_debug_renderer.hpp"
#include "utils/tools.hpp"

void ParkDebugRenderer::maybeSave(bool debug, bool saveImages, cv::Mat &image,
                                  cv::Mat &ipmImage)
{
    if (!debug || !saveImages)
        return;
    const auto now = std::chrono::steady_clock::now();
    if (lastSavedAt.time_since_epoch().count() != 0 &&
        now - lastSavedAt < std::chrono::milliseconds(500))
        return;
    lastSavedAt = now;
    savePicture("../res/samples/imgs/", ipmImage);
    savePicture("../res/samples/imgs/", image);
}
