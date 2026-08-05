#include "icar.hpp"

void Icar::sendVehicleCommand(FrameCycle &frame)
{
    if (params->config.debug && frame.frameAvailable)
    {
        detection->drawBox(frame.image, params->results);
        center->drawImage(params, frame.image);
        motion->drawImage(params, frame.image);
        show->setNewWindow(3, "Ctrl", frame.image);

        Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3);
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

    if (!params->config.debug || frame.exitRequested || frame.cameraTimedOut)
        client->carControl(params->ctrl.speed, params->ctrl.servo);
}
