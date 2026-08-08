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
 * @file state.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 机器人状态信息
 * @version 0.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <algorithm>
#include <atomic>
#include <string>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "utils/json.hpp"
#include "ctrl/track.hpp"
#include "config/config.hpp"
#include "config/config_loader.hpp"
#include "runtime/fsm_mode.hpp"
#include "runtime/path_override.hpp"
#include "runtime/planner_safety.hpp"
#include "runtime/yfork_phase.hpp"
#include "ctrl/control_geometry.hpp"
#include "utils/config_validation.hpp"

using namespace std;

struct Control
{
    bool stop = false;
    bool back = false;
    bool slow = false;
    bool obstacleSlow = false;
    uint16_t servo = PWMSERVOMID;
    float speed = 0.0f;
    int center = COLSIMAGE / 2;
    float laneHeadingCorrection = 0.0f;
    // 单边模式下横向P/D权重（Center按LaneRecoveryMode写入；非单边=1.0）。
    float laneLateralScale = 1.0f;
    int laneLateralScaleReason = 0; // LateralScaleReason 枚举值（遥测诊断）
    // 姿态控制诊断：lateral/heading拆分后的实际作用量（Motion写入）。
    float lateralRaw = 0.0f;
    float lateralApplied = 0.0f;
    float headingApplied = 0.0f;
    int pwmDiff = 0;
    vector<PointX> centerEdge;
    int lineArea = 0;
    bool parking = false;
    int countAcc = 500;
    int startupSteeringCount = 500;
    bool yforkReset = false;
};

/**
 * @brief FSM状态场景
 *
 */
enum class Feature
{
    FORK,
    PARK,
    BUSY,
    SLOW,
    STOP,
    CROSS,
    YFORK,
    STATION,
    OBSTACLE
};

/**
 * @brief 车辆状态参数（FSM共享传递）
 *
 */
struct Params
{
public:
    control_algorithms::StopReasonState stopReasons;
    void setStopReason(control_algorithms::StopReason reason, bool active)
    {
        stopReasons.set(reason, active);
    }
    bool hasStopReason(control_algorithms::StopReason reason) const
    {
        return stopReasons.has(reason);
    }
    bool mustStop() const { return stopReasons.mustStop(); }
    bool mustStopExcept(control_algorithms::StopReason reason) const
    {
        return stopReasons.mustStopExcept(reason);
    }
    string stopReasonString() const { return stopReasons.string(); }
    /**
     * @brief Construct a new Params object
     *
     */
    Params(const std::string &configPath = "../res/config.json")
    {
        config = loadConfig(configPath);

        validateConfig();

        mode = FsmMode::NORMAL;                    // 初始化控制模式
        modeLast = FsmMode::NORMAL;                // 初始化控制模式
        track = make_shared<Track>();              // 赛道线处理
        track->rowCutUp = config.rowCutUp;         // 图像顶部切行（前瞻距离）
        track->rowCutBottom = config.rowCutBottom;
        track->maxGapRows = config.maxGapRows; // 图像底部切行（盲区距离）
        track->singleLaneInteriorPointsMin = config.singleLaneInteriorPointsMin;

        // 初始化圈数
        totalLaps = std::max(1, std::min(config.totalLaps, 3));
        currentLap = 1;

        // 初始化第一圈配置
        updateLapConfig();
    };
    ~Params() {};

    Control ctrl;                       // 车辆控制指令(实时)
    Config config;                      // 系统配置
    FsmMode mode, modeLast;             // FSM状态场景
    shared_ptr<Track> track;            // 赛道识别类
    PathOverride pathOverride;
    PlannerSafetyState plannerSafety;
    GeometryPolicy geometryPolicy{GeometryPolicy::PERCEPTION_ALLOWED};
    uint64_t pathFrameId{0};
    std::vector<PredictResult> results; // AI推理结果
    bool aiResultFresh = false;         // 本控制帧是否收到了一组新的AI结果
    int totalLaps;                      // 总圈数
    int currentLap;                     // 当前圈数
    std::atomic<bool> manualTakeover{false}; // 手动接管模式（跨 AI/主线程共享）
    bool stationStopCompleted = false;  // station已完成一次停车
    bool stationStarted = false;        // station已触发检测（pressTimer启动）
    YforkRuntimePhase yforkPhase{YforkRuntimePhase::INACTIVE};
    int yforkBranch = 0;                // yfork分支：0=无, 1=左, 2=右
    bool busyZone = false;              // 施工区标志（station据此调整检测参数）
    bool takeoverJustEnded = false;     // 手动接管刚结束
    int autoRecoveryFrames = 0;         // Automatic-control recovery hold
    bool laneSafetyStop = false;        // Latch FSM while lane recovery is incomplete
    int alertCountdown = 0;             // 蜂鸣器报警倒计时（帧数）
    int alertDecelCount = 0;            // 报警目标减速倒计时（帧数）
    int busyAlertCountdown = 0;         // 施工区蜂鸣器倒计时（帧数）
    bool lapTaskRequired = false;       // 当前圈是否配置了必须完成的主任务
    bool lapTaskCompleted = false;      // 当前圈主任务是否已可靠完成

public:
    void validateConfig() const
    {
        validateIcarConfig(config, ROWSIMAGE);
    }

    void beginPathOverride(PathSource source)
    {
        pathOverride.clear();
        pathOverride.setEdges(source, track->pointsEdgeLeft,
                              track->pointsEdgeRight);
    }

    void clearPathOverride(PathSource source)
    {
        pathOverride.clear(source);
    }

    void releasePlannerSafety(PathSource source)
    {
        plannerSafety.clear(source);
        setStopReason(control_algorithms::StopReason::PLANNER,
                      plannerSafety.latched);
    }

    void reconcilePlannerSafetyWithMode()
    {
        if (plannerSafety.latched && !pathSourceAllowed(
                plannerSafety.rejectedSource, mode))
            plannerSafety.clear();
        setStopReason(control_algorithms::StopReason::PLANNER,
                      plannerSafety.latched);
    }

    void advancePathFrame()
    {
        pathOverride.tick(++pathFrameId);
    }

    bool dropPathOverrideIfDisallowed(FsmMode currentMode)
    {
        if (!pathOverride.active() ||
            pathSourceAllowed(pathOverride.source, currentMode))
            return false;
        const PathSource staleSource = pathOverride.source;
        std::cout << "[Path] Dropped stale " << pathSourceName(staleSource)
                  << " path in " << fsmModeName(currentMode) << " mode."
                  << std::endl;
        pathOverride.clear(staleSource);
        return true;
    }

    const Config::LapConfig &activeLapConfig() const
    {
        return *config.currentLapConfig;
    }

    bool featureEnabled(Feature feature) const
    {
        const auto &lap = activeLapConfig();
        switch (feature)
        {
        case Feature::FORK: return lap.fork;
        case Feature::PARK: return lap.park;
        case Feature::BUSY: return lap.busy;
        case Feature::SLOW: return lap.slow;
        case Feature::STOP: return lap.stop;
        case Feature::CROSS: return lap.cross;
        case Feature::YFORK: return lap.yfork;
        case Feature::STATION: return lap.station;
        case Feature::OBSTACLE: return lap.obstacle;
        }
        return false;
    }

    /**
     * @brief 更新当前圈配置
     */
    void updateLapConfig()
    {
        switch (currentLap)
        {
        case 1:
            config.currentLapConfig = &config.lap1;
            break;
        case 2:
            config.currentLapConfig = &config.lap2;
            break;
        case 3:
            config.currentLapConfig = &config.lap3;
            break;
        default:
            config.currentLapConfig = nullptr;
            break;
        }

        lapTaskRequired = config.currentLapConfig &&
                          (config.currentLapConfig->park ||
                           config.currentLapConfig->busy ||
                           config.currentLapConfig->yfork);
        lapTaskCompleted = false;
    }

    void completeLapTask(const char *source)
    {
        if (!lapTaskRequired || lapTaskCompleted)
            return;
        lapTaskCompleted = true;
        printf("[Task] Lap %d completed by %s\n", currentLap, source);
    }

    /**
     * @brief 切换到下一圈
     */
    void nextLap()
    {
        if (currentLap < totalLaps)
        {
            currentLap++;
            updateLapConfig();
        }
    }

private:
};
