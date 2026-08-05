/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file busy.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmBusy::FsmBusy(std::shared_ptr<Params> par)
    : FSMState(FsmMode::BUSY, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmBusy::~FsmBusy()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmBusy::getMode()
{
    if (enable && params->config.busy && params->config.currentLapConfig->busy)
    {
        if (manualTakeover)
        {
            return FsmMode::MANUAL;
        }
        else if (waitingForTakeover)
        {
            return FsmMode::BUSY_WAIT;
        }
        else
        {
            return FsmMode::BUSY;
        }
    }
    return FsmMode::NORMAL;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmBusy::run(Mat &img)
{
    if (!params->config.busy) // 该模式未启用
        return;
    // 手动接管恢复期递减
    enable = false;
    // 行驶通过施工区模式（退出手动接管后）：左转标志触发退出转向，和出停车场一致
    if (drivingThrough)
    {
        if (!exiting)
        {
            // 施工区停靠模式：等待station完成停车后再检测左转
            bool stationActive = params->stationStarted && !params->stationStopCompleted;
            bool waitStationStop = false;
            if (params->config.currentLapConfig &&
                params->config.currentLapConfig->busyStopEnable &&
                params->config.currentLapConfig->busyStopPoint > 0)
            {
                if (!params->stationStopCompleted)
                {
                    waitStationStop = true; // 正在停车，等待
                }
                else if (params->config.currentLapConfig->busyStopPoint == 1 &&
                         stationExitCooldown < 30)
                {
                    // 第一个框：停车后等1秒再检测左转
                    stationExitCooldown++;
                    waitStationStop = true;
                }
            }

            if (!stationActive && !waitStationStop)
            {
                // 等待检测左转标志，触发退出转向
                for (int i = 0; i < params->results.size(); i++)
                {
                    if (params->results[i].type == LABEL_LEFT &&
                        params->results[i].width < 100 &&
                        params->results[i].height < 120 &&
                        (params->results[i].y + params->results[i].height / 2) > ROWSIMAGE * 0.27)
                    {
                        exiting = true;
                        exitStartedAt = std::chrono::steady_clock::now();
                        countRes = 0;
                        printf("[Busy] Left sign detected, starting exit turn\n");
                        break;
                    }
                }
            }
        }

        if (exiting)
        {
            // 每帧重绘左转车道线（步骤[04]的赛道识别会覆盖，所以必须在runFsm中重新设置）
            params->track->pointsEdgeLeft.clear();
            params->track->pointsEdgeRight.clear();
            PointX startL(ROWSIMAGE - 10, 1);
            PointX endL(ROWSIMAGE / 3, 1);
            PointX midL((startL.x + endL.x) * 0.3, (startL.y + endL.y) / 2);
            params->track->pointsEdgeLeft = Bezier(0.008, {startL, midL, endL});
            PointX startR(ROWSIMAGE - 10, int(COLSIMAGE * 0.8));
            PointX endR(ROWSIMAGE / 3, 5);
            PointX midR((startR.x + endR.x) * 0.3, (startR.y + endR.y) / 2);
            params->track->pointsEdgeRight = Bezier(0.008, {startR, midR, endR});

            // 左转标志消失即完成退出
            bool leftVisible = false;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_LEFT &&
                    params->results[i].width < 100 &&
                    params->results[i].height < 120)
                {
                    leftVisible = true;
                    break;
                }
            }
            if (params->aiResultFresh)
            {
                if (leftVisible)
                    countRes = 0;
                else
                    countRes++;
            }

            const bool exitTimedOut =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - exitStartedAt).count() >= 2000;
            if (countRes > 2 || exitTimedOut)
            { // 标志丢失或超时
                const bool exitConfirmed = countRes > 2;
                drivingThrough = false;
                exiting = false;
                enable = false;

                busyConfirmation = control_algorithms::BusyConfirmationState{};

                countRes = 0;
                params->busyZone = false;  // 施工区结束
                params->ctrl.countAcc = 0; // 出库缓加速，约1.7s后恢复正常速度
                params->track->pointsEdgeLeft.clear();
                params->track->pointsEdgeRight.clear();
                if (exitConfirmed)
                    params->completeLapTask("construction-exit");
                else
                    printf("[Busy] Exit timed out; lap task remains incomplete\n");
                printf("[Busy] Exit turn complete, returning to normal mode\n");
                return;
            }
        }
        enable = true; // 保持施工区模式
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
        {
            startManualTakeover();
        }
        else
        {
            slowing = true;
            drivingThrough = true;
            waitingForTakeover = false;
            params->setStopReason(control_algorithms::StopReason::BUSY, false);
            timeout = 0;
            cout << "[Busy] Automatic construction traversal active" << endl;
        }
    }

    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        params->track->pointsEdgeRight.size() < ROWSIMAGE / 2)
        return;
    if (slowing)
    {
        timeout++;
        enable = true; // 场景检测使能标志
        if (timeout > 10)
            slowing = false;
    }

    // 手动接管模式下不执行自动控制
    if (manualTakeover)
    {
        return;
    }

}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmBusy::show(Mat &img)
{
    if (!enable || params->mode != FsmMode::BUSY)
        return;

    putText(img, "[1] Busy", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 启动手动接管
 */
void FsmBusy::startManualTakeover()
{
    params->setStopReason(control_algorithms::StopReason::BUSY, false);
    params->setStopReason(control_algorithms::StopReason::MANUAL, true);
    manualTakeover = true;
    waitingForTakeover = false;
    printf("[Busy] Manual takeover active. Pass the obstacle and press R/RETURN before the first station box.\n");
}

/**
 * @brief 结束手动接管
 */
void FsmBusy::endManualTakeover()
{
    params->setStopReason(control_algorithms::StopReason::MANUAL, false);
    params->setStopReason(control_algorithms::StopReason::BUSY, false);
    busyConfirmation = control_algorithms::BusyConfirmationState{};
    manualTakeover = false;
    waitingForTakeover = false; // 改为false，不进入BUSY_WAIT
    enable = true;              // 保持在施工区模式
    params->busyZone = true; // 标记施工区状态

    // 重置停车相关变量
    // 使用station的skip逻辑控制停靠点，busy只负责行驶通过模式
    drivingThrough = true;
    if (params->config.currentLapConfig &&
        params->config.currentLapConfig->busyStopEnable &&
        params->config.currentLapConfig->busyStopPoint > 0)
    {
        printf("[Busy] Station skip controlled, driving through (stopPoint=%d)\n",
               params->config.currentLapConfig->busyStopPoint);
    }
    else
    {
        printf("[Busy] No parking, driving through construction zone\n");
    }

    // 重置退出相关变量
    exiting = false;
    countRes = 0;
    stationExitCooldown = 0;
    printf("[Busy] Manual takeover ended\n");
}

void FsmBusy::resetLap()
{
    params->setStopReason(control_algorithms::StopReason::BUSY, false);
    params->setStopReason(control_algorithms::StopReason::MANUAL, false);
    enable = false;

    busyConfirmation = control_algorithms::BusyConfirmationState{};
    manualTakeover = false;
    waitingForTakeover = true;
    timeout = 0;
    drivingThrough = false;
    exiting = false;
    countRes = 0;
    stationExitCooldown = 0;

    // 停靠区停车状态复位
    params->busyZone = false;
    params->stationStopCompleted = false;
}
