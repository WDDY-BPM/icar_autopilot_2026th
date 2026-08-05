#include "icar.hpp"

void Icar::preprocessFrame(FrameCycle &frame)
{
        //[02] 图像存储
        if (params->config.saveImg && !params->config.debug) // 存储原始图像
            savePicture(frame.image);
        else if (params->config.saveImg && params->config.debug) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        predeal->correction(frame.image); // 图像矫正
        // Publish an unannotated, geometrically corrected frame before lane,
        // AI and FSM processing. Overlay coordinates therefore match exactly.
        if (fsmFactory.manual->isConnected())
            frame.frameId = fsmFactory.manual->sendImage(
                frame.image, &frame.timestampMs);
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        {
            std::lock_guard<std::mutex> lock(mtxImg);
            imgShare = frame.image.clone();
            imgShareFrameId = frame.frameId;
            readyImg = true;
        }
        cvImg.notify_one();
        /*-------------------------------------------------------*/
        frame.binary = predeal->binaryzation(frame.image); // 图像二值化
        //[04] Track recognition (skipped during manual takeover).
        frame.lanesUpdated =
            !fsmFactory.busy->isInManualTakeover();
        if (frame.lanesUpdated)
        {
            const auto containsFork = [](const std::vector<PredictResult> &results) {
                return std::any_of(results.begin(), results.end(),
                    [](const PredictResult &result) { return result.type == LABEL_FORK; });
            };
            bool forkMarkerActive = containsFork(params->results);
            {
                std::lock_guard<std::mutex> resultLock(mtxRes);
                if (readyRes)
                    forkMarkerActive = containsFork(latestResults);
            }
            params->track->allowOuterEnvelope = !forkMarkerActive;
            params->track->handle(frame.binary);
            const bool widthLearningMode = params->track->quality.valid &&
                !params->manualTakeover && !params->ctrl.fitting &&
                !forkMarkerActive &&
                params->mode == FsmMode::NORMAL;
            center->observeLaneWidth(params->track->pointsEdgeLeft,
                                     params->track->pointsEdgeRight,
                                     widthLearningMode);
        }
        if (params->config.debug)
        {
            show->setNewWindow(1, "Bin", frame.binary);
            cv::Mat imgTrack = frame.image.clone();
            params->track->drawImage(imgTrack); // 图像绘制赛道识别结果
            show->setNewWindow(2, "Track", imgTrack);
            if (params->config.saveIpm && params->config.saveImg)
            {
                cv::Mat imgIpm;
                ipm.homography(imgTrack, imgIpm);
                savePicture(imgIpm); // 保存图像
            }
        }
}
