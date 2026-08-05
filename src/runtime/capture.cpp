#include "icar.hpp"

bool Icar::acquireFrame(FrameCycle &frame)
{
        //[01] 视频源读取
        if (params->config.debug) // 综合显示调试UI窗口
        {
            if (show->indexLast == show->index) // 图像帧未更新
            {
                if (client->keypress.exchange(false))
                {
                    client->buzzerSound(client->BUZZER_FINISH); // 祖传提示音效
                    printf("-----> System Exit!!! <-----\n");
                    client->carControl(0.0f, PWMSERVOMID);
                    exit(0); // 调试退出前先停车
                }
                show->show();        // 显示综合绘图
                client->sendHeart(); // 发送给服务器在线心跳
                usleep(10 * 1000);   // us延迟
                return false;
            }

            frameCapture->set(cv::CAP_PROP_POS_FRAMES, show->index);
            if (!frameCapture->read(frame.image))
            {
                params->setStopReason(control_algorithms::StopReason::CAMERA, true);
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
                previousFinalServo = PWMSERVOMID;
                motion->reset();
                client->carControl(0.0f, PWMSERVOMID);
                return false;
            }
            show->indexLast = show->index;
        }
        else
        {
            if (!frameCapture->waitForLatest(
                    frame.image, std::chrono::milliseconds(150)))
            {
                if (!params->hasStopReason(control_algorithms::StopReason::CAMERA))
                    std::cout << "[Camera] Frame timeout; CAMERA stop active." << std::endl;
                params->setStopReason(control_algorithms::StopReason::CAMERA, true);
                cameraFreshFrames = 0;
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
                previousFinalServo = PWMSERVOMID;
                motion->reset();
                client->carControl(0.0f, PWMSERVOMID);
                return false;
            }
        }
        if (params->hasStopReason(control_algorithms::StopReason::CAMERA))
        {
            if (++cameraFreshFrames >= 3)
            {
                params->setStopReason(control_algorithms::StopReason::CAMERA, false);
                cameraFreshFrames = 0;
                std::cout << "[Camera] Fresh frames recovered; CAMERA stop cleared." << std::endl;
            }
        }
    return true;
}
