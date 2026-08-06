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
 * @file park.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车场控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/park.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmPark::FsmPark(std::shared_ptr<Params> par)
    : FSMState(FsmMode::PARK, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmPark::~FsmPark()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmPark::getMode()
{
    // std::cout << "[Park::getMode] step=" << (int)state.stage
    //           << " currentLapConfig->park=" << params->config.currentLapConfig->park
    //           << " parkSpot=" << params->config.currentLapConfig->parkSpot
    //           << " currentLap=" << params->currentLap << std::endl;

    if (state.stage == Step::NONE || !params->featureEnabled(Feature::PARK))
    {
        params->ctrl.parking = false;
        return FsmMode::NORMAL;
    }
    else
    {
        params->ctrl.parking = true;
        return FsmMode::PARK;
    }
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmPark::run(Mat &img)
{
    const auto now = std::chrono::steady_clock::now();
    updateGeometryPolicy();
    const ParkObservation observation = params->aiResultFresh
        ? scanParkObservation(params->results) : ParkObservation{};
    if (!params->featureEnabled(Feature::PARK)) // 该模式未启用
    {
        params->clearPathOverride(PathSource::PARK);
        return;
    }
    if (params->mustStopExcept(control_algorithms::StopReason::PLANNER) &&
        !params->hasStopReason(control_algorithms::StopReason::PARK))
        return;

    stopping = false; // 停车等待标志
    if (params->ctrl.stop || waiting) // 禁行区不进行车库图像处理
    {
        if (params->ctrl.stop)
        {
            if (state.stage == Step::ENABLE || state.stage == Step::FORKIN)
            {
                stopping = true; // 停车等待标志
            }
        }
        else if (waiting) // 等待前车入库
        {
            stopping = true; // 停车等待标志
            if (!waitTimerActive)
            {
                state.waitStartedAt = now;
                waitTimerActive = true;
            }
            if (now - state.waitStartedAt >= std::chrono::seconds(7))
            {
                waiting = false;
                waitTimerActive = false;
            }
        }
    }

    dispatchStage(observation, now);

}


void FsmPark::dispatchStage(const ParkObservation &observation,
                           std::chrono::steady_clock::time_point now)
{
    switch (state.stage)
    {
    case Step::NONE: handleNone(observation, now); break;
    case Step::ENABLE: handleEnable(observation, now); break;
    case Step::FORKIN: handleForkIn(observation, now); break;
    case Step::TRACKIN: handleTrackIn(observation, now); break;
    case Step::ENTER: handleEnter(observation, now); break;
    case Step::PARKING: handleParking(observation, now); break;
    case Step::WAIT_PICKUP: handleWaitPickup(observation, now); break;
    case Step::EXIT: handleExit(observation, now); break;
    case Step::TRACKOUT: handleTrackOut(observation, now); break;
    case Step::FORKOUT: handleForkOut(observation, now); break;
    }
}

void FsmPark::handleNone(const ParkObservation &observation,
                         std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleEnable(const ParkObservation &observation,
                           std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleForkIn(const ParkObservation &observation,
                           std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleTrackIn(const ParkObservation &observation,
                            std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleEnter(const ParkObservation &observation,
                          std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleParking(const ParkObservation &observation,
                            std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleWaitPickup(const ParkObservation &observation,
                               std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleExit(const ParkObservation &observation,
                         std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleTrackOut(const ParkObservation &observation,
                             std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }
void FsmPark::handleForkOut(const ParkObservation &observation,
                            std::chrono::steady_clock::time_point now)
{ dispatchStageLegacy(observation, now); }

void FsmPark::dispatchStageLegacy(const ParkObservation &observation,
                                  std::chrono::steady_clock::time_point now)
{
    switch (state.stage)
    {
    case Step::NONE: // AI未识别
    {
        if (!params->aiResultFresh)
            break;

        bool parkDetectedThisFrame = false;
        for (const auto &marker : observation.parkMarkers)
        {
            if (marker.height < 100 && marker.width < 90 &&
                marker.x > COLSIMAGE / 2) // AI标志：停车场
            {
                parkDetectedThisFrame = true;
                break;
            }
        }
        if (parkDetectedThisFrame)
            state.aiEvidenceFrames++;

        if (state.aiEvidenceFrames >= 3)
        {
            setStep(Step::ENABLE); // 设置停车场新步骤
            std::cout << "[Park] Parking area confirmed" << std::endl;
        }

        if (state.aiEvidenceFrames > 0) // 识别AI标志后开始场次计数
        {
            state.aiMissingFrames++;
            if (state.aiMissingFrames >= 5)
            {
                state.aiMissingFrames = 0;
                state.aiEvidenceFrames = 0;
            }
        }
        break;
    }

    case Step::ENABLE: // 停车场使能
    {
        if (!params->ctrl.stop) // 停车等待
            state.stageControlFrames++;
        if (state.stageControlFrames > 80) // 超时退出停车状态
            reset();      // 停车场数据复位

        if (state.stage == Step::NONE)
            return;
        if (params->aiResultFresh)
        {
            state.aiMissingFrames++;
            if (!observation.parkMarkers.empty())
                state.aiMissingFrames = 0;

            const PredictResult &fork = observation.bestChoice;
            if (fork.score > 0)
            {
                state.aiMissingFrames = 0;
                if ((fork.y + fork.height / 2) > ROWSIMAGE * 0.35)
                    state.aiEvidenceFrames++;
            }
        }

        if (state.aiMissingFrames > 30 || state.aiEvidenceFrames > 1)
            setStep(Step::FORKIN);

        // parkSpot为0时穿过停车场不停车（不提前回退，让流程走到FORKIN）
        break;
    }

    case Step::FORKIN: // 入库岔路转向
    {
        state.stageControlFrames++;
        replanTracking();                             // 车道线重绘（岔路左转）
        if (observation.hasGate)
            state.aiEvidenceFrames++;
        if (state.aiEvidenceFrames > 2 || state.stageControlFrames > 24) // 转向超时：
        {
            spots.reset(); // 车位信息复位

            // 自动标记非目标车位为已占用（parkSpot指定的是目标空车位）
            for (int i = 1; i <= 4; i++)
            {
                if (i != params->config.currentLapConfig->parkSpot)
                    spots.counter[i - 1] = 5; // counter > 4 即标记为已占用
            }

            countOut = 0;
            setStep(Step::TRACKIN); // 设置停车场新步骤
        }

        if (params->aiResultFresh && state.stageControlFrames > 20)
        {
            if (!observation.forkMarkers.empty()) // 搜索AI标志：岔路箭头
                state.aiEvidenceFrames++;
        }
        break;
    }

    case Step::TRACKIN: // 入库巡线中
    {
        state.stageControlFrames++;  // 超时计数
        state.aiMissingFrames++; // 控制周期计数（仅用于超时，不作为AI证据）

        //[01] 道闸检测
        if (observation.hasGate)
        {
            const auto &gate = observation.gate;
            if (gate.width > 100 && gate.height < 130)
            {
                if ((gate.y + gate.height) > ROWSIMAGE * 0.4) // 停车距离计算
                {
                    stopping = true; // 停车等待标志
                    break;
                }

                // 出停车场检测
                if (params->aiResultFresh &&
                    (gate.y + gate.height) > ROWSIMAGE * 0.25 &&
                    spots.checked) // 道闸距离估算
                {
                    countOut++;
                    if (countOut > 4)
                        setStep(Step::TRACKOUT); // 设置停车场新步骤

                    break;
                }
                if (params->aiResultFresh)
                {
                    state.stageControlFrames = 0;
                    state.aiEvidenceFrames = 0;
                }
            }
        }

        //[02] 控制中心重规划
        spots.forks = findParkStation(observation.forkMarkers);
        PredictResult resLeft = observation.bestLeft;
        if (spots.forks.size() > 0 || resLeft.score > 0)
        {
            int targetSpot = params->config.currentLapConfig->parkSpot;
            bool rightSide = (targetSpot == 3 || targetSpot == 4);

            PredictResult direction;
            if (resLeft.score > 0)
                direction = resLeft;
            else
            {
                // 根据目标车位选择对应侧的叉（解决巡线时左偏）
                if (rightSide && spots.forks.size() >= 2)
                {
                    // 选右侧的叉
                    float cx0 = spots.forks[0].x + spots.forks[0].width / 2;
                    float cx1 = spots.forks[1].x + spots.forks[1].width / 2;
                    direction = (cx0 > cx1) ? spots.forks[0] : spots.forks[1];
                }
                else if (!rightSide && spots.forks.size() >= 2)
                {
                    // 选左侧的叉
                    float cx0 = spots.forks[0].x + spots.forks[0].width / 2;
                    float cx1 = spots.forks[1].x + spots.forks[1].width / 2;
                    direction = (cx0 < cx1) ? spots.forks[0] : spots.forks[1];
                }
                else
                {
                    direction = spots.forks[0];
                }
            }

            PointX start = PointX(ROWSIMAGE - 20, COLSIMAGE / 2); // 补线起点：车头
            if (countIn < 50)                                     // 起点矫正
            {
                if (params->track->pointsEdgeLeft.size() > ROWSIMAGE / 5 && params->track->pointsEdgeRight.size() > ROWSIMAGE / 5)
                {
                    start = {(params->track->pointsEdgeLeft[0].x + params->track->pointsEdgeRight[0].x) / 2,
                             (params->track->pointsEdgeLeft[0].y + params->track->pointsEdgeRight[0].y) / 2};
                }
                countIn++;
            }
            params->pathOverride = ParkPathPlanner::buildTrackGuide(
                start, direction, rightSide ? 40 : -40, params->config);
            state.stageControlFrames = 0;                                          // 超时计数
            state.aiEvidenceFrames = 0;

            // 出停车场检测
            if (params->aiResultFresh && resLeft.score > 0 &&
                (resLeft.y + resLeft.height / 2) > ROWSIMAGE * 0.2)
                countOut++;
            if (countOut > 3)
                setStep(Step::TRACKOUT); // 设置停车场新步骤
        }

        //[03] 空闲车位检测
        if (spots.forks.size() == 2) // 当AI图像同时检测到两个岔路箭头时判断车位是否空闲
        {
            // 目标车位来自圈次配置，非目标车位由FORKIN初始化为占用，
            // 不进行车辆检测（避免假车/他车误分类到目标车位；PredictResult默认未初始化，
            // carPark.score为随机值会导致所有车位被误判为已占用）
            for (int i = 0; i < 4; i++)
            {
                if (spots.counter[i] > 4)
                    spots.spotEnable[i] = false; // 停车位已占
            }

            // 驶入车位编号确认（用较小阈值使checked尽早完成，让spotUp/spotDown控制入库时机）
            float fork0_center = spots.forks[0].y + spots.forks[0].height / 2;
            float fork1_center = spots.forks[1].y + spots.forks[1].height / 2;
            float threshold = ROWSIMAGE * spotDown;
            if (params->aiResultFresh &&
                (fork0_center > threshold || fork1_center > threshold))
            {
                spots.countRes++;
            }
            else if (params->aiResultFresh)
                spots.countRes = 0;
            if (spots.countRes > 1) // 开始确认驶入车位号
            {
                spots.checked = true;
                spots.countRes = 0;
                std::cout << "[Park] Spots checked = true!" << std::endl;
            }
        }

        //[04] 入库检测
        if (params->aiResultFresh && spots.checked)
        {
            // 远处车位 1/4号（左前/右前）→ 第一个叉
            if (spots.spotEnable[0] || spots.spotEnable[3])
            {
                if (spots.forks.size() > 0)
                {
                    if ((spots.forks[0].y + spots.forks[0].height / 2) > ROWSIMAGE * spotUp)
                    {
                        spots.times++;
                        if (spots.times > 0)
                        {
                            spots.times = 0;
                            setStep(Step::ENTER);
                            pointsEdgeLeftPast.clear();
                            pointsEdgeRightPast.clear();
                        }
                    }
                }
            }
            // 近处车位 2/3号（左后/右后）→ 第二个叉
            else if (spots.spotEnable[1] || spots.spotEnable[2])
            {
                if (spots.forks.size() == 2)
                {
                    if ((spots.forks[1].y + spots.forks[1].height / 2) > ROWSIMAGE * spotDown)
                    {
                        spots.times++;
                        if (spots.times > 0)
                        {
                            spots.times = 0;
                            setStep(Step::ENTER);
                            pointsEdgeLeftPast.clear();
                            pointsEdgeRightPast.clear();
                        }
                    }
                }
            }
        }

        // 出库状态切换
        if (state.aiMissingFrames > 100 && state.stageControlFrames > 23)
            setStep(Step::FORKOUT); // 纯控制周期超时，不混入AI证据计数器
        break;
    }

    case Step::ENTER: // 驶入停车位
    {
        state.stageControlFrames++;
        if (state.stageControlFrames < 18) // 入库转向
        {
            if (spots.spotEnable[0] || spots.spotEnable[1])      // 1/2号车位（左侧）
                replanTracking(true);                            // 入库车道线重绘（左转）
            else if (spots.spotEnable[2] || spots.spotEnable[3]) // 3/4号车位（右侧）
                replanTracking(false);                           // 入库车道线重绘（右转）
        }
        else
        {
            // Track重新捕获正常车道线
            int height = 0;
            if (params->track->pointsEdgeLeft.size() > COLSIMAGE / 2 && params->track->pointsEdgeRight.size() > COLSIMAGE / 2)
            {
                for (int i = 1; i <= 10; i++)
                {
                    height += params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - i].x;
                    height += params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - i].x;
                }
                height = height / 20;
            }

            if (height > ROWSIMAGE * 0.3) // 入库结束
            {
                spots.countRes++;
                if (spots.countRes > 2)
                {
                    setStep(Step::PARKING); // 停车完成
                    break;
                }
            }

            // 入库直行
            // 左车道线
            params->pathOverride =
                ParkPathPlanner::buildInSpotStraight(params->config);
        }

        if (params->pathOverride.validFor(PathSource::PARK) &&
            params->pathOverride.hasLeft() &&
            params->pathOverride.hasRight())
        {
            pointsEdgeLeftPast.push_back(params->pathOverride.leftEdge);
            pointsEdgeRightPast.push_back(params->pathOverride.rightEdge);
        }

        if (state.stageControlFrames > 30)           // 转向超时
            setStep(Step::PARKING); // 停车完成
        break;
    }

    case Step::PARKING: // 停车
    {
        params->setStopReason(control_algorithms::StopReason::PARK, true);
        if (now - state.stageStartedAt >= std::chrono::milliseconds(700))
            setStep(Step::WAIT_PICKUP);

        break;
    }

    case Step::WAIT_PICKUP: // 等待乘客上车
    {
        params->setStopReason(control_algorithms::StopReason::PARK, true);
        if (now - state.stageStartedAt >= std::chrono::seconds(3))
        {
            printf("[Park] Pickup wait complete, exiting parking spot\n");
            setStep(Step::EXIT);
        }

        break;
    }

    case Step::EXIT: // 出库
    {
        params->setStopReason(control_algorithms::StopReason::PARK, false);
        params->ctrl.back = true;  // 倒车

        if (pointsEdgeLeftPast.size() < 1 || pointsEdgeRightPast.size() < 1)
            setStep(Step::TRACKOUT); // 停车完成
        else
        {
            // 出库轨迹复现
            params->pathOverride = ParkPathPlanner::buildReplay(
                pointsEdgeLeftPast.back(), pointsEdgeRightPast.back(),
                params->config);
            pointsEdgeLeftPast.pop_back(); // 删除最后一组路径
            pointsEdgeRightPast.pop_back();
        }

        break;
    }

    case Step::TRACKOUT: // 出库巡线
    {
        state.stageControlFrames++; // 超时计数
        //[01] 道闸检测
        if (observation.hasGate)
        {
            const auto &gate = observation.gate;
            if (gate.width > 100 && gate.height < 130)
            {
                if (params->aiResultFresh)
                {
                    state.stageControlFrames = 0;
                    state.aiEvidenceFrames = 0;
                }
                if ((gate.y + gate.height) > ROWSIMAGE * 0.4) // 停车距离计算
                {
                    stopping = true; // 停车等待标志
                    break;
                }
            }
        }

        //[02] 控制中心重规划
        spots.forks = findParkStation(observation.forkMarkers);
        PredictResult resLeft = observation.bestLeft;

        if (spots.forks.size() > 0 || resLeft.score > 0)
        {
            PredictResult direction;
            if (resLeft.score > 0)
                direction = resLeft;
            else
                direction = spots.forks[0];
            PointX start = PointX(ROWSIMAGE - 20, COLSIMAGE / 2);                // 补线起点：车头
            params->pathOverride = ParkPathPlanner::buildTrackGuide(
                start, direction, 0, params->config);
            if (params->aiResultFresh && spots.forks.size() > 0)
                state.stageControlFrames = 0; // 定时入库关闭 | 仅依靠AI标志转向

            if (params->aiResultFresh && resLeft.score > 0 &&
                (resLeft.y + resLeft.height / 2) > ROWSIMAGE * 0.35)
            {
                state.aiEvidenceFrames++;
            }
        }

        // 出库状态切换
        if (state.stageControlFrames > 48 || state.aiEvidenceFrames > 2)
            setStep(Step::FORKOUT); // 设置停车场新步骤

        break;
    }

    case Step::FORKOUT: // 出库岔路转向
    {
        state.stageControlFrames++;
        replanTracking(); // 车道线重绘（岔路左转）

        // 搜索左转标志
        const bool leftSign = observation.hasLeft &&
            observation.bestLeft.width < 100 && observation.bestLeft.height < 120;
        if (params->aiResultFresh)
        {
            if (leftSign) // 标志未丢失
                state.aiEvidenceFrames = 0;
            else
                state.aiEvidenceFrames++;
        }

        const bool exitTimedOut =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - state.forkOutStartedAt).count() >= 2000;
        if (exitTimedOut || state.aiEvidenceFrames > 2) // 转向超时
        {
            const bool exitConfirmed = state.aiEvidenceFrames > 2;
            params->ctrl.countAcc = params->config.startupRampFrames;        // 跳过缓加速，直接恢复速度
            params->ctrl.yforkReset = true;    // 通知yfork复位，防止残留forkSeen误触发
            if (exitConfirmed)
                params->completeLapTask("park-exit");
            else
                printf("[Park] Exit timed out; lap task remains incomplete\n");
            setStep(Step::NONE);               // 设置停车场新步骤
        }
        break;
    }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmPark::show(Mat &img)
{
    if (params->mode != FsmMode::PARK)
        return;

    putText(img, "[5] Park", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    // 绘制边缘点
    for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeLeft[i].y, params->track->pointsEdgeLeft[i].x), 2,
               Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeRight[i].y, params->track->pointsEdgeRight[i].x), 2,
               Scalar(0, 255, 255), -1); // 黄色点
    }

    for (int i = 0; i < params->ctrl.centerEdge.size(); i++)
        circle(img, Point(params->ctrl.centerEdge[i].y, params->ctrl.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);

    string str = "Enable";
    switch (state.stage)
    {
    case Step::FORKIN:
        str = "Forkin";
        break;
    case Step::TRACKIN:
    {
        str = "Trackin";
        for (int i = 0; i < spots.forks.size(); i++) // 绘制停车位箭头标志
        {
            putText(img, to_string(i + 1), Point(spots.forks[i].x + spots.forks[i].width / 2, spots.forks[i].y + spots.forks[i].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
            cv::Rect rect(spots.forks[i].x, spots.forks[i].y, spots.forks[i].width, spots.forks[i].height);
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }

        // 透视变换视角
        Mat imgIpm = Mat::zeros(Size(COLSIMAGEIPM, ROWSIMAGEIPM), CV_8UC3); // 创建全黑图像
        // 绘制所有4个车位标签（实际布局：左前=1，左后=2，右后=3，右前=4）
        // 绿色=空闲(spotEnable=true)，红色=占用(spotEnable=false)
        if (spots.forks.size() > 0)
        {
            Scalar c1 = spots.spotEnable[0] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "1", Point(spots.forks[0].x + spots.forks[0].width / 2, spots.forks[0].y + spots.forks[0].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c1, 0.5);
        }
        if (spots.forks.size() > 1)
        {
            Scalar c2 = spots.spotEnable[1] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "2", Point(spots.forks[1].x + spots.forks[1].width / 2, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c2, 0.5);

            Scalar c3 = spots.spotEnable[2] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "3", Point(spots.forks[1].x + spots.forks[1].width / 2 + 100, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c3, 0.5);

            Scalar c4 = spots.spotEnable[3] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "4", Point(COLSIMAGE - 31, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c4, 0.5);
        }
        debugRenderer.maybeSave(params->config.debug, params->config.saveImg,
                                img, imgIpm);
        break;
    }
    case Step::ENTER:
        str = "Enter";
        break;
    case Step::PARKING:
        str = "Parking";
        break;
    case Step::WAIT_PICKUP:
        str = "Wait pickup";
        break;
    case Step::EXIT:
        str = "Exit";
        break;
    case Step::TRACKOUT:
        str = "TRACKOUT";
        break;
    case Step::FORKOUT:
        str = "Forkout";
        break;
    default:
        break;
    }
    putText(img, "PARK - " + str, Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 停车场数据复位
 *
 */
void FsmPark::reset()
{
    params->clearPathOverride(PathSource::PARK);
    params->releasePlannerSafety(PathSource::PARK);
    state = ParkStateData{};
    spots.reset();
    pointsEdgeLeftPast.clear();
    pointsEdgeRightPast.clear();
    stopping = false;
    params->setStopReason(control_algorithms::StopReason::PARK, false);
    params->geometryPolicy = GeometryPolicy::PERCEPTION_ALLOWED;
    state.aiMissingFrames = 0;      // AI场景识别计数器
    state.stageControlFrames = 0;       // 超时计数器
    state.stage = Step::NONE; // 停车步骤
    waiting = false;   // 停车等待使能
    waitTimerActive = false;
    countOut = 0;      // 出库检测计数
    countIn = 0;       // 入库矫正计数器
    params->ctrl.back = false;
    params->ctrl.parking = false;
}

/**
 * @brief 设置下阶段
 *
 * @param state.stage
 */
void FsmPark::setStep(Step st)
{
    params->clearPathOverride(PathSource::PARK);
    state.aiEvidenceFrames = 0; // AI场景识别计数器
    state.aiMissingFrames = 0; // 场次计数器
    state.stageControlFrames = 0;  // 超时计数器
    state.stage = st;    // 停车步骤
    state.stageStartedAt = std::chrono::steady_clock::now();
    params->setStopReason(control_algorithms::StopReason::PARK,
        st == Step::PARKING || st == Step::WAIT_PICKUP);
    if (st == Step::NONE || st == Step::ENABLE ||
        st == Step::PARKING || st == Step::WAIT_PICKUP)
        params->releasePlannerSafety(PathSource::PARK);
    if (st == Step::FORKOUT)
        state.forkOutStartedAt = std::chrono::steady_clock::now();
    countIn = 0;  // 入库矫正计数器
    params->ctrl.back = false; // 倒车失能
    updateGeometryPolicy();
}

void FsmPark::updateGeometryPolicy()
{
    switch (state.stage)
    {
    case Step::FORKIN:
    case Step::ENTER:
    case Step::EXIT:
    case Step::FORKOUT:
        params->geometryPolicy = GeometryPolicy::PLANNED_REQUIRED;
        break;
    case Step::PARKING:
    case Step::WAIT_PICKUP:
        params->geometryPolicy = GeometryPolicy::STOPPED;
        break;
    default:
        params->geometryPolicy = GeometryPolicy::PERCEPTION_ALLOWED;
        break;
    }
}

/**
 * @brief 车道线重绘（岔路左转）
 *
 */
void FsmPark::replanTracking()
{
    params->pathOverride = state.stage == Step::FORKIN
        ? ParkPathPlanner::buildForkIn(params->config)
        : ParkPathPlanner::buildForkOut(params->config);
}
/**
 * @brief 停车入库车道线重绘
 *
 * @param left true：左转 | false：右转
 */
void FsmPark::replanTracking(bool left)
{
    params->pathOverride = ParkPathPlanner::buildParkingTurn(left, params->config);
}

/**
 * @brief 搜索停车位坐标
 *
 * @param results
 */
vector<PredictResult> FsmPark::findParkStation(const vector<PredictResult> &results) const
{
    return selectParkStations(results);
}

void FsmPark::resetLap()
{
    params->setStopReason(control_algorithms::StopReason::PARK, false);
    reset();
    spots.reset();
    pointsEdgeLeftPast.clear();
    pointsEdgeRightPast.clear();
    stopping = false;
    state.aiEvidenceFrames = 0;
}
