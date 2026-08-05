#include "icar.hpp"

void Icar::runModel()
    {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this]
                   { return readyImg.load() || shuttingDown.load(); });
        if (shuttingDown)
            return;
        cv::Mat img = imgShare.clone(); // 图像拷贝出来再释放锁
        const uint64_t inferenceFrameId = imgShareFrameId;
        readyImg = false;
        lock.unlock();

        // 手动接管期间跳过AI推理
        if (!params->manualTakeover)
        {
            try
            {
                detection->inference(img);
                std::lock_guard<std::mutex> lock_result(mtxRes);
                latestResults = detection->results;
                latestResultsFrameId = inferenceFrameId;
                latestResultsPublishedAtMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                readyRes = true;
            }
            catch (const std::exception &error)
            {
                std::cerr << "[AI] Inference failed: " << error.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "[AI] Inference failed with unknown error" << std::endl;
            }
        }
    }
