#include "runtime/latest_frame_capture.hpp"

LatestFrameCapture::~LatestFrameCapture()
{
    stop();
}

bool LatestFrameCapture::open(const std::string &source, int backend)
{
    stop();
    capture_ = backend == cv::CAP_ANY
        ? std::make_shared<cv::VideoCapture>(source)
        : std::make_shared<cv::VideoCapture>(source, backend);
    return capture_->isOpened();
}

bool LatestFrameCapture::isOpened() const
{
    return capture_ && capture_->isOpened();
}

bool LatestFrameCapture::set(int property, double value)
{
    return capture_ && capture_->set(property, value);
}

double LatestFrameCapture::get(int property) const
{
    return capture_ ? capture_->get(property) : 0.0;
}

bool LatestFrameCapture::read(cv::Mat &frame)
{
    return capture_ && capture_->read(frame);
}

void LatestFrameCapture::start()
{
    if (!isOpened() || running_.exchange(true))
        return;
    thread_ = std::thread(&LatestFrameCapture::captureLoop, this);
}

bool LatestFrameCapture::waitForLatest(
    cv::Mat &frame, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(frameMutex_);
    const bool available = frameReady_.wait_for(lock, timeout, [this]() {
        return !running_ || latestSequence_ != consumedSequence_;
    });
    if (!available || latestSequence_ == consumedSequence_)
        return false;
    frame = latestFrame_.clone();
    consumedSequence_ = latestSequence_;
    return !frame.empty();
}

void LatestFrameCapture::stop()
{
    running_ = false;
    frameReady_.notify_all();
    if (thread_.joinable())
        thread_.join();
}

void LatestFrameCapture::captureLoop()
{
    while (running_)
    {
        cv::Mat frame;
        if (!capture_->read(frame))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            latestFrame_ = std::move(frame);
            ++latestSequence_;
        }
        frameReady_.notify_one();
    }
}
