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
 * @file station.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停靠站停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/station.hpp"

FsmStation::FsmStation(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STATION, par)
{
}

FsmStation::~FsmStation()
{
}

FsmMode FsmStation::getMode()
{
    if (step != Step::NONE || pressTimer > 0)
        return FsmMode::STATION;
    else
        return FsmMode::NORMAL;
}

void FsmStation::run(Mat &img)
{
    // 手动接管刚结束，重置计数
    if (params->takeoverJustEnded)
    {
        params->takeoverJustEnded = false;
        detectedBoxIndex = 0;
        boxArmed = true;
        boxMissingFrames = 0;
        printf("[Station] Manual takeover ended, reset counters\n");
    }

    // 手动接管期间不做任何识别
    if (params->manualTakeover)
        return;

    // 当前圈未启用station
    if (!params->config.station || !params->config.currentLapConfig->station)
        return;

    countInit++;
    if (countInit > 999)
        countInit = 999;
    else if (countInit < 60) // 发车屏蔽
        return;

    switch (step)
    {
    case Step::NONE:
    {
        // 施工区未启用停车或busyStopPoint为0时跳过检测
        if (params->busyZone && (!params->config.currentLapConfig->busyStopEnable ||
                                 params->config.currentLapConfig->busyStopPoint == 0))
            break;

        // 非施工区复位跳过计数
        if (!params->busyZone)
        {
            detectedBoxIndex = 0;
            boxArmed = true;
            boxMissingFrames = 0;
        }

        // 停车后冷却期内不检测
        if (cooldown > 0)
        {
            cooldown--;
            break;
        }

        if (pressTimer > 0)
        {
            pressTimer++;
            // 施工区第一个框1.3s(40帧)，第二个目标框0.6s(20帧)，左岔路1.1s(33帧)，其他0.6s(19帧)
            int pressThreshold = 19;
            if (params->busyZone && detectedBoxIndex <= 1)
                pressThreshold = 40;
            if (params->busyZone && params->config.currentLapConfig->busyStopPoint > 1 &&
                detectedBoxIndex >= params->config.currentLapConfig->busyStopPoint)
                pressThreshold = 20;
            if (params->yforkBranch == 1)
                pressThreshold = 33;
            if (pressTimer > pressThreshold)
            {
                printf("[Station] Pressed +%.1fs, stopping...\n", pressThreshold / 30.0f);
                setStep(Step::STOP);
            }
            break;
        }

        // 旧AI结果可能被多个摄像头控制帧重复使用；框计数和消失计数
        // 只能由一组新的推理结果推进。
        if (!params->aiResultFresh)
            break;

        // 左岔路：引导结束见框开始计时（要求框到底部0.5以下，给足时间完成左转）
        if (params->yforkBranch == 1)
        {
            // 引导期间不检测
            if (params->yforkGuiding)
                break;

            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_STATION)
                {
                    int boxCx = params->results[i].x + params->results[i].width / 2;
                    int boxBottom = params->results[i].y + params->results[i].height;
                    if (boxCx < COLSIMAGE / 2 && boxBottom > ROWSIMAGE * 0.5)
                    {
                        params->stationStarted = true;
                        pressTimer = 1;
                        printf("[Station] Left branch, seen, 0.3s stop\n");
                        break;
                    }
                }
            }
        }
        else
        {
            // yfork引导期间不检测station框，避免干扰岔路导航
            if (params->yforkGuiding)
                break;

            if (params->busyZone)
            {
                bool stationVisible = false;
                bool boxAtThreshold = false;
                for (const auto &result : params->results)
                {
                    if (result.type != LABEL_STATION)
                        continue;
                    stationVisible = true;
                    if (result.y + result.height > static_cast<int>(ROWSIMAGE * 0.5))
                        boxAtThreshold = true;
                }

                // 只关心已经到达计数阈值的框。前一框离开阈值区域后，
                // 即使远处的下一框仍可见，也应允许重新武装。
                if (!boxAtThreshold)
                {
                    if (++boxMissingFrames >= 8)
                    {
                        boxArmed = true;
                        boxMissingFrames = 8;
                    }
                }
                else
                {
                    boxMissingFrames = 0;
                }

                if (!params->stationStopCompleted && stationVisible && boxAtThreshold && boxArmed)
                {
                    const int targetBox = params->config.currentLapConfig->busyStopPoint;
                    detectedBoxIndex++;
                    boxArmed = false;
                    printf("[Station] Counted box #%d (target=%d)\n", detectedBoxIndex, targetBox);
                    if (detectedBoxIndex == targetBox)
                    {
                        params->stationStarted = true;
                        pressTimer = 1;
                        printf("[Station] Target box reached\n");
                    }
                }
            }
            else
            {
                for (const auto &result : params->results)
                {
                    if (result.type == LABEL_STATION &&
                        result.y + result.height > ROWSIMAGE - 10)
                    {
                        params->stationStarted = true;
                        pressTimer = 1;
                        printf("[Station] Pressed\n");
                        break;
                    }
                }
            }
        }
        break;
    }

    case Step::STOP:
    {
        params->ctrl.stop = true;
        stopCounter++;
        printf("[Station] Stop %d/90\n", stopCounter);
        if (stopCounter > 90) // 施工区乘客下车等待约3秒
        {
            printf("[Station] Stop end, resume\n");
            params->stationStopCompleted = true; // 通知yfork边线突变可以退出了
            setStep(Step::NONE);
            cooldown = params->busyZone ? 6 : 150; // 施工区0.2秒冷却，其他5秒
        }
        break;
    }
    }
}

void FsmStation::show(Mat &img)
{
    if (params->mode != FsmMode::STATION)
        return;

    putText(img, "[10] Station", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::STOP:
        putText(img, "[10] Station - STOP", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }
}

void FsmStation::setStep(Step st)
{
    step = st;
    stopCounter = 0;
    pressTimer = 0;
    params->ctrl.stop = false;
}

void FsmStation::resetLap()
{
    setStep(Step::NONE);
    countInit = 0;
    cooldown = 0;
    detectedBoxIndex = 0;
    boxArmed = true;
    boxMissingFrames = 0;
    params->stationStopCompleted = false;
    params->stationStarted = false;
}
