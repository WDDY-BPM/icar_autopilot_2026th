#include "icar.hpp"

bool Icar::acquireFrame(FrameCycle &frame)
{
    if (params->config.debug)
    {
        if (show->indexLast == show->index)
        {
            if (client->keypress.exchange(false))
            {
                client->buzzerSound(client->BUZZER_FINISH);
                printf("-----> System Exit!!! <-----\n");
                frame.exitRequested = true;
                params->setStopReason(control_algorithms::StopReason::EMERGENCY, true);
            }
            show->show();
            client->sendHeart();
            usleep(10 * 1000);
            return true;
        }

        frameCapture->set(cv::CAP_PROP_POS_FRAMES, show->index);
        if (!frameCapture->read(frame.image))
        {
            const auto recovery = cameraRecovery.onTimeout();
            params->setStopReason(control_algorithms::StopReason::CAMERA, true);
            frame.cameraTimedOut = true;
            frame.cameraReady = recovery.controlReady;
            return true;
        }
        show->indexLast = show->index;
    }
    else if (!frameCapture->waitForLatest(
                 frame.image, std::chrono::milliseconds(150)))
    {
        if (!params->hasStopReason(control_algorithms::StopReason::CAMERA))
            std::cout << "[Camera] Frame timeout; CAMERA stop active." << std::endl;
        const auto recovery = cameraRecovery.onTimeout();
        params->setStopReason(control_algorithms::StopReason::CAMERA, true);
        frame.cameraTimedOut = true;
        frame.cameraReady = recovery.controlReady;
        return true;
    }

    frame.frameAvailable = true;
    const auto recovery = cameraRecovery.onFreshFrame();
    frame.cameraReady = recovery.controlReady;
    params->setStopReason(control_algorithms::StopReason::CAMERA,
                          recovery.cameraStopActive);
    if (!recovery.cameraStopActive &&
        recovery.freshFrames == CameraRecoveryState::REQUIRED_FRESH_FRAMES &&
        recovery.holdFrames == CameraRecoveryState::RECOVERY_HOLD_FRAMES)
        std::cout << "[Camera] Three fresh frames received; holding control recovery."
                  << std::endl;
    return true;
}
