#include "fsm/stop.hpp"

FsmStop::FsmStop(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STOP, par)
{
}

FsmStop::~FsmStop() = default;

FsmMode FsmStop::getMode()
{
    if (step == Step::NONE || !params->featureEnabled(Feature::STOP))
        return FsmMode::NORMAL;
    return FsmMode::STOP;
}

void FsmStop::run(Mat &img)
{
    (void)img;
    if (!params->featureEnabled(Feature::STOP))
        return;

    switch (step)
    {
    case Step::NONE:
        if (!params->aiResultFresh)
            break;
        if (std::any_of(params->results.begin(), params->results.end(),
                [](const PredictResult &result) { return result.type == LABEL_GATE; }))
            ++countRec;
        if (countRec > 2)
            setStep(Step::ENABLE);
        else if (countRec > 0 && ++countSes > 4)
            setStep(Step::NONE);
        break;

    case Step::ENABLE:
    {
        ++timeout;
        if (params->aiResultFresh)
        {
            ++countSes;
            for (const auto &result : params->results)
            {
                if (result.type == LABEL_GATE)
                {
                    countSes = 0;
                    timeout = 0;
                    if (result.y + result.height > ROWSIMAGE * 0.4)
                        ++countRec;
                    break;
                }
            }
        }
        if (countRec > 2)
            setStep(Step::STOP);
        else if (countSes >= 10)
            setStep(Step::NONE);
        else if (timeout > 50)
        {
            setStep(Step::STOP);
            params->setStopReason(control_algorithms::StopReason::GATE, true);
            params->ctrl.speed = 0.0f;
        }
        break;
    }

    case Step::STOP:
        params->setStopReason(control_algorithms::StopReason::GATE, true);
        if (!params->aiResultFresh)
            break;
        ++countSes;
        if (std::any_of(params->results.begin(), params->results.end(),
                [](const PredictResult &result) { return result.type == LABEL_GATE; }))
            countSes = 0;
        if (countSes >= 30)
        {
            setStep(Step::NONE);
            params->setStopReason(control_algorithms::StopReason::GATE, false);
        }
        break;
    }
}

void FsmStop::show(Mat &img)
{
    if (params->mode != FsmMode::STOP)
        return;
    putText(img, "[7] Stop", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
    if (step == Step::ENABLE)
        putText(img, "[7] STOP - ENABLE", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
    else if (step == Step::STOP)
        putText(img, "[7] STOP - STOPPING", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
}

void FsmStop::setStep(Step next)
{
    step = next;
    params->setStopReason(control_algorithms::StopReason::GATE,
                          next == Step::STOP);
    countRec = 0;
    countSes = 0;
    timeout = 0;
}

void FsmStop::resetLap()
{
    params->setStopReason(control_algorithms::StopReason::GATE, false);
    setStep(Step::NONE);
}
