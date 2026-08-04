#pragma once
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
 * @file motion.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 运动控制器
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include "utils/tools.hpp"
#include "utils/params.hpp"
#include "ctrl/control_algorithms.hpp"

using namespace std;

class Motion
{

public:
    /**
     * @brief 姿态PD控制器
     *
     * @param center 智能车控制中心
     */
    void poseControl(shared_ptr<Params> &params, float dtSeconds)
    {
        const float dt = sanitizeDt(dtSeconds);
        const float rawError = params->ctrl.center - COLSIMAGE / 2.0f;

        if (!controlInitialized)
        {
            // Start from the measured error. Starting at zero delayed the first
            // useful steering response by several frames.
            filteredError = rawError;
            errorLast = rawError;
            controlInitialized = true;
        }
        else
        {
            const float filterAlpha = 1.0f - std::exp(-dt / params->config.steeringFilterTau);
            filteredError += filterAlpha * (rawError - filteredError);
        }

        const float maxErrorStep = params->config.maxErrorRate * dt;
        const float error = std::clamp(filteredError,
                                       errorLast - maxErrorStep,
                                       errorLast + maxErrorStep);
        const float currentP = std::abs(error) * params->config.runP2 +
                               params->config.runP1;
        const float derivative = (error - errorLast) / dt;
        const int pwmDiff = static_cast<int>(std::lround(
            error * currentP + derivative * params->config.turnD +
            params->ctrl.laneHeadingCorrection));

        // Clamp in the signed domain before converting to uint16_t. Automatic
        // and manual steering share the same time-based actuator rate limiter.
        int targetServo = std::clamp(PWMSERVOMID - pwmDiff,
                                     PWMSERVOMIN, PWMSERVOMAX);
        float servoRate = params->config.servoRate;
        if (params->ctrl.startupSteeringCount < params->config.startupRampFrames)
        {
            targetServo = control_algorithms::applyStartupServoLimit(
                targetServo, PWMSERVOMID, params->config.startupServoLimit,
                params->ctrl.startupSteeringCount, params->config.startupRampFrames);
            servoRate = params->config.startupServoRate;
            params->ctrl.startupSteeringCount++;
        }
        errorLast = error;
        params->ctrl.servo = limitServoCommand(targetServo, dt, servoRate);
    }

    uint16_t limitServoCommand(int targetServo, float dtSeconds,
                                 float servoRatePerSecond = 750.0f)
    {
        const float dt = sanitizeDt(dtSeconds);
        const int maxServoStep = std::max(1, static_cast<int>(
            std::ceil(servoRatePerSecond * dt)));
        targetServo = std::clamp(targetServo, PWMSERVOMIN, PWMSERVOMAX);
        targetServo = std::clamp(targetServo,
                                 lastServo - maxServoStep,
                                 lastServo + maxServoStep);
        lastServo = targetServo;
        return static_cast<uint16_t>(targetServo);
    }

    uint16_t syncServoCommand(int servo)
    {
        lastServo = std::clamp(servo, PWMSERVOMIN, PWMSERVOMAX);
        return static_cast<uint16_t>(lastServo);
    }

    void resetControl()
    {
        controlInitialized = false;
        filteredError = 0.0f;
        errorLast = 0.0f;
    }

    void reset()
    {
        resetControl();
        lastServo = PWMSERVOMID;
    }
    /**
     * @brief 变加速控制
     *
     * @param params
     */
    void speedControl(shared_ptr<Params> &params)
    {
        if (params->ctrl.stop)
        {
            params->ctrl.speed = 0.0f;
            return;
        }
        if (params->ctrl.back)
        {
            params->ctrl.speed = -params->config.velPark;
            return;
        }

        float desiredSpeed = params->config.velLow;
        if (params->mode == FsmMode::STOP)
            desiredSpeed = params->config.velStop;
        else if (params->mode == FsmMode::PARK)
            desiredSpeed = params->config.velPark;
        else if (params->mode == FsmMode::CROSS)
            desiredSpeed = params->config.velCross;
        else if (params->mode == FsmMode::STATION)
            desiredSpeed = params->config.velSlow;
        else if (params->mode == FsmMode::BUSY || params->busyZone)
            desiredSpeed = params->config.velBusy;
        else if (params->mode == FsmMode::CURVE)
            desiredSpeed = params->config.velCurve;
        else if (params->mode == FsmMode::YFORK)
            desiredSpeed = params->config.velYfork;
        else if (params->ctrl.slow || params->ctrl.obstacleSlow)
            desiredSpeed = params->config.velSlow;
        else
        {
            const auto curveSpeed =
                control_algorithms::calculateCenterlineSpeed(
                    params->ctrl.centerEdge, params->config.velHigh,
                    params->config.velCurve);
            desiredSpeed = curveSpeed.speed;
        }

        // Apply the launch envelope last, so every autonomous mode shares it.
        params->ctrl.speed = control_algorithms::applyStartupSpeed(
            desiredSpeed, params->ctrl.countAcc,
            params->config.startupRampFrames, params->config.startupSpeed);

        if (params->alertDecelCount > 0)
            params->ctrl.speed = std::max(0.0f, params->ctrl.speed - 0.1f);
    }
    /**
     * @brief 显示赛道线识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(shared_ptr<Params> &params, Mat &img)
    {
        std::string str = "Vel: " + doble2String(params->ctrl.speed, 2);
        putText(img, str, Point(COLSIMAGE - 100, 120), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 速度
    }

    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    void outlineCheck(shared_ptr<Params> &params)
    {
        // 停车场模式下车辆主动脱离赛道线，不检测
        if (params->ctrl.parking)
        {
            countRes = 0;
            outline = false;
            return;
        }

        // 出库后冷却期，给车辆时间重新捕获赛道线
        if (params->ctrl.outlineCooldown > 0)
        {
            params->ctrl.outlineCooldown--;
            countRes = 0;
            outline = false;
            return;
        }

        if (outline) // 已出现
        {
            params->ctrl.stop = true; // 停车
            countRes++;
            if (countRes > 15)
            {
                std::cout << "-----> [Stop] Game over, system exit!!! <-----" << std::endl;
                std::_Exit(0); // 直接退出，跳过静态析构避免ONNX崩溃
            }
        }
        else // 出线检测
        {
            if (params->track->pointsEdgeLeft.size() < 30 &&
                params->track->pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
            {
                countRes++;
                countOut = 0;
                if (countRes > 30)
                    outline = true;
            }
            else
            {
                countRes = 0; // 边线恢复立即归零，避免跨圈/跨路段累加
                countOut = 0;
            }
        }
    }

private:
    static float sanitizeDt(float dtSeconds)
    {
        return std::clamp(dtSeconds, 0.005f, 0.1f);
    }

    bool controlInitialized = false;
    float filteredError = 0.0f;
    float errorLast = 0.0f;
    int lastServo = PWMSERVOMID;
    int countRes = 0;
    int countOut = 0;
    bool outline = false; // 出线标志
};
