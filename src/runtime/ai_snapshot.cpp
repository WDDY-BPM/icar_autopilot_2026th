#include "icar.hpp"

void Icar::consumeAiSnapshot(FrameCycle &frame)
{
        // 在主线程按帧获取AI结果快照；FSM不再与推理线程共享可变vector
        frame.receivedNewAiResult = false;
        std::int64_t receivedAiResultPublishedAtMs = -1;
        {
            std::lock_guard<std::mutex> resultLock(mtxRes);
            if (readyRes)
            {
                params->results = latestResults;
                activeResultsFrameId = latestResultsFrameId;
                receivedAiResultPublishedAtMs = latestResultsPublishedAtMs;
                readyRes = false;
                frame.receivedNewAiResult = true;
            }
        }

        params->aiResultFresh = frame.receivedNewAiResult;
        frame.manualBeforeFsm = fsmFactory.busy->isInManualTakeover();
        params->manualTakeover = frame.manualBeforeFsm;
        frame.aiStale = updateAiSafety(!frame.manualBeforeFsm, frame.receivedNewAiResult,
                                      receivedAiResultPublishedAtMs);

        frame.startupGateReleased = updateStartupGate(frame.receivedNewAiResult);

        // Alert timers advance once per control frame, independently of FSM returns.
        updateAlerts();
}
