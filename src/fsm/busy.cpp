#include "fsm/busy.hpp"

FsmBusy::FsmBusy(std::shared_ptr<Params> par)
    : FSMState(FsmMode::BUSY, par)
{
}

FsmBusy::~FsmBusy() = default;

FsmMode FsmBusy::getMode()
{
    if (!enable || !params->featureEnabled(Feature::BUSY))
        return FsmMode::NORMAL;
    if (manualTakeover)
        return FsmMode::MANUAL;
    if (waitingForTakeover)
        return FsmMode::BUSY_WAIT;
    return FsmMode::BUSY;
}

void FsmBusy::run(Mat &img)
{
    (void)img;
    if (!params->featureEnabled(Feature::BUSY))
    {
        params->clearPathOverride(PathSource::BUSY);
        params->releasePlannerSafety(PathSource::BUSY);
        return;
    }

    enable = false;
    if (drivingThrough)
    {
        const bool stationRequired = params->config.currentLapConfig &&
            params->config.currentLapConfig->busyStopEnable &&
            params->config.currentLapConfig->busyStopPoint > 0;

        bool leftVisible = false;
        if (params->aiResultFresh)
        {
            leftVisible = std::any_of(params->results.begin(), params->results.end(),
                [](const PredictResult &result) {
                    return result.type == LABEL_LEFT && result.width < 100 &&
                        result.height < 120 &&
                        result.y + result.height / 2 > ROWSIMAGE * 0.27;
                });
        }
        const auto event = exitState.update(
            stationRequired, params->stationStopCompleted,
            params->aiResultFresh, leftVisible,
            std::chrono::steady_clock::now());
        if (event == BusyExitEvent::EXIT_STARTED)
            printf("[Busy] Left sign detected, starting exit turn\n");

        if (exitState.exiting)
        {
            PointX startL(ROWSIMAGE - 10, 1);
            PointX endL(ROWSIMAGE / 3, 1);
            PointX midL((startL.x + endL.x) * 0.3, (startL.y + endL.y) / 2);
            PointX startR(ROWSIMAGE - 10, int(COLSIMAGE * 0.8));
            PointX endR(ROWSIMAGE / 3, 5);
            PointX midR((startR.x + endR.x) * 0.3, (startR.y + endR.y) / 2);
            params->pathOverride.setEdges(
                PathSource::BUSY,
                Bezier(0.008, {startL, midL, endL}),
                Bezier(0.008, {startR, midR, endR}),
                0.65f, params->config.velBusy, 2);
        }

        if (event == BusyExitEvent::STATION_WAIT_TIMEOUT ||
            event == BusyExitEvent::SIGN_WAIT_TIMEOUT ||
            event == BusyExitEvent::EXIT_GUIDE_TIMEOUT ||
            event == BusyExitEvent::TRAVERSAL_TIMEOUT)
        {
            params->setStopReason(control_algorithms::StopReason::BUSY, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            params->clearPathOverride(PathSource::BUSY);
            params->releasePlannerSafety(PathSource::BUSY);
            if (event == BusyExitEvent::STATION_WAIT_TIMEOUT)
                printf("[Busy] Station wait timeout; vehicle stopped; lap task remains incomplete.\n");
            else if (event == BusyExitEvent::SIGN_WAIT_TIMEOUT)
                printf("[Busy] Exit sign timeout; vehicle stopped.\n");
            else if (event == BusyExitEvent::TRAVERSAL_TIMEOUT)
                printf("[Busy] Construction traversal timeout; vehicle stopped.\n");
            else
                printf("[Busy] Exit guidance timeout; vehicle stopped; lap task remains incomplete.\n");
            enable = true;
            return;
        }
        else if (event == BusyExitEvent::COMPLETED)
        {
            drivingThrough = false;
            enable = false;
            busyConfirmation = control_algorithms::BusyConfirmationState{};
            params->busyZone = false;
            params->ctrl.countAcc = 0;
            params->clearPathOverride(PathSource::BUSY);
            params->releasePlannerSafety(PathSource::BUSY);
            params->completeLapTask("construction-exit");
            printf("[Busy] Exit turn complete, returning to normal mode\n");
            return;
        }
        enable = true;
        return;
    }

    bool busyDetected = false;
    if (params->aiResultFresh)
    {
        busyDetected = std::any_of(params->results.begin(), params->results.end(),
            [](const PredictResult &result) {
                return result.type == LABEL_BUSY &&
                    result.height < 100 && result.width < 80;
            });
    }
    const auto confirmationEvent = control_algorithms::updateBusyConfirmation(
        busyConfirmation, params->aiResultFresh, busyDetected);
    if (confirmationEvent == control_algorithms::BusyConfirmationEvent::WAITING)
    {
        if (!drivingThrough)
        {
            params->setStopReason(control_algorithms::StopReason::BUSY, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
        enable = true;
        if (!params->busyZone)
            params->busyAlertCountdown = 40;
        params->busyZone = true;
        cout << "[Busy] Construction candidate stopped; waiting for fresh confirmation" << endl;
    }
    else if (confirmationEvent == control_algorithms::BusyConfirmationEvent::CLEARED)
    {
        params->setStopReason(control_algorithms::StopReason::BUSY, false);
        params->busyZone = false;
        enable = false;
        cout << "[Busy] Construction candidate rejected by fresh AI results" << endl;
    }
    else if (confirmationEvent == control_algorithms::BusyConfirmationEvent::CONFIRMED)
    {
        enable = true;
        cout << "[Busy] Construction zone confirmed" << endl;
        if (params->config.currentLapConfig &&
            params->config.currentLapConfig->manualTakeover && !drivingThrough)
            startManualTakeover();
        else
        {
            slowing = true;
            drivingThrough = true;
            waitingForTakeover = false;
            params->setStopReason(control_algorithms::StopReason::BUSY, false);
            timeout = 0;
            const bool stationRequired = params->config.currentLapConfig &&
                params->config.currentLapConfig->busyStopEnable &&
                params->config.currentLapConfig->busyStopPoint > 0 &&
                !params->stationStopCompleted;
            exitState.startDriving(
                std::chrono::steady_clock::now(), stationRequired);
            cout << "[Busy] Automatic construction traversal active" << endl;
        }
    }

    if (slowing)
    {
        ++timeout;
        enable = true;
        if (timeout > 10)
            slowing = false;
    }
}

void FsmBusy::show(Mat &img)
{
    if (enable && params->mode == FsmMode::BUSY)
        putText(img, "[1] Busy", Point(COLSIMAGE / 2 - 50, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

void FsmBusy::startManualTakeover()
{
    params->setStopReason(control_algorithms::StopReason::BUSY, false);
    params->setStopReason(control_algorithms::StopReason::MANUAL, true);
    params->clearPathOverride(PathSource::BUSY);
    params->releasePlannerSafety(PathSource::BUSY);
    exitState.reset();
    manualTakeover = true;
    waitingForTakeover = false;
    printf("[Busy] Manual takeover active. Pass the obstacle and press R/RETURN before the first station box.\n");
}

void FsmBusy::endManualTakeover()
{
    params->setStopReason(control_algorithms::StopReason::MANUAL, false);
    params->setStopReason(control_algorithms::StopReason::BUSY, false);
    params->clearPathOverride(PathSource::BUSY);
    params->releasePlannerSafety(PathSource::BUSY);
    busyConfirmation = control_algorithms::BusyConfirmationState{};
    manualTakeover = false;
    waitingForTakeover = false;
    enable = true;
    params->busyZone = true;
    drivingThrough = true;
    slowing = false;
    timeout = 0;
    const bool stationRequired = params->config.currentLapConfig &&
        params->config.currentLapConfig->busyStopEnable &&
        params->config.currentLapConfig->busyStopPoint > 0 &&
        !params->stationStopCompleted;
    exitState.startDriving(
        std::chrono::steady_clock::now(), stationRequired);
    printf("[Busy] Manual takeover ended; automatic construction traversal active\n");
}

void FsmBusy::resetLap()
{
    params->setStopReason(control_algorithms::StopReason::BUSY, false);
    params->setStopReason(control_algorithms::StopReason::MANUAL, false);
    params->clearPathOverride(PathSource::BUSY);
    params->releasePlannerSafety(PathSource::BUSY);
    enable = false;
    busyConfirmation = control_algorithms::BusyConfirmationState{};
    manualTakeover = false;
    waitingForTakeover = true;
    timeout = 0;
    slowing = false;
    drivingThrough = false;
    exitState.reset();
    params->busyZone = false;
    params->stationStopCompleted = false;
}
