#include "icar.hpp"

void Icar::sendVehicleCommand(FrameCycle &frame)
{
        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(frame.image, params->results); // 使用主线程结果快照绘框
            center->drawImage(params, frame.image); // 图像绘制控制路径
            motion->drawImage(params, frame.image); // 图像绘制速度
            show->setNewWindow(3, "Ctrl", frame.image);

            // 特殊区域图像处理结果显示
            Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
            fsmFactory.busy->show(imgRes);
            fsmFactory.park->show(imgRes);
            fsmFactory.stop->show(imgRes);
            fsmFactory.cross->show(imgRes);
            fsmFactory.fork->show(imgRes);
            fsmFactory.slow->show(imgRes);
            fsmFactory.obstacle->show(imgRes);
            fsmFactory.yfork->show(imgRes);
            show->setNewWindow(4, "FSM", imgRes);
        }
        else // 实车控制
        {
            // 无论手动/自动模式，每帧发送控制指令（MCU需要持续PWM更新）
            client->carControl(params->ctrl.speed, params->ctrl.servo);
        }
}
