#pragma once
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
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
 * @file icar.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"
#include "fsm/manualControl.hpp"
#include "fsm/fork.hpp"
#include "fsm/park.hpp"
#include "fsm/stop.hpp"
#include "fsm/cross.hpp"
#include "fsm/yfork.hpp"
#include "fsm/station.hpp"
#include "fsm/slow.hpp"
#include "fsm/obstacle.hpp"
#include "com/client.hpp"
#include "utils/detection.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"
#include "runtime/latest_frame_capture.hpp"
#include "runtime/camera_recovery.hpp"
#include "runtime/final_command.hpp"

using namespace std;
using namespace cv;

class Icar
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;               // 避障控制
        shared_ptr<FsmPark> park;               // 停车场控制
        shared_ptr<FsmStop> stop;               // 停车区控制
        shared_ptr<FsmCross> cross;             // 斑马线停车控制
        shared_ptr<FsmFork> fork;               // 停车场岔路控制
        shared_ptr<FsmSlow> slow;               // 慢行区控制
        shared_ptr<FsmObstacle> obstacle;       // 全局障碍物检测（锥桶/行人）
        shared_ptr<FsmYfork> yfork;             // Y型岔路口控制
        shared_ptr<FsmStation> station;         // 停靠站控制
        shared_ptr<ManualControlThread> manual; // 手动接管控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<LatestFrameCapture> frameCapture;
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Client> client;            // TCP客户端通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器

    int lastLap = 0; // 上一圈号（检测圈变更时复位FSM）
    bool emergencyStopWasActive = false;
    int previousFinalServo = PWMSERVOMID;
    control_algorithms::SingleLaneSpeedLimitState singleLaneSpeedLimit;
    control_algorithms::LaneUnconfirmedState laneUnconfirmedState;
    std::chrono::steady_clock::time_point lastOverlayBuilt{};
    uint64_t lastTelemetryFrameId{0};
    std::int64_t lastTelemetryTimestampMs{0};
    control_algorithms::AiFreshnessState aiFreshness;

    // 全局共享数据链
    enum class StartupGateState
    {
        WAIT_FOR_CONE,
        WAIT_FOR_REMOVAL,
        RELEASED
    };
    StartupGateState startupGateState = StartupGateState::WAIT_FOR_CONE;
    int startupConeSeenCount = 0;
    int startupConeMissingCount = 0;
    int startupLaneValidCount = 0;
    int startupDiagnosticFrames = 0;
    bool startupEnvironmentChecked = false;
    bool startupConeDetected = false;
    CameraRecoveryState cameraRecovery;
    std::atomic<bool> shutdownRequested{false};

    bool updateAiSafety(bool automaticMode, bool successfulFreshResult,
                        std::int64_t successfulResultMs = -1);
    bool updateStartupGate(bool receivedNewAiResult);

    cv::Mat imgShare;
    uint64_t imgShareFrameId{0};
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};
    std::atomic<bool> shuttingDown{false};
    std::mutex mtxRes;
    std::atomic<bool> readyRes{false};
    std::vector<PredictResult> latestResults;
    uint64_t latestResultsFrameId{0};
    std::int64_t latestResultsPublishedAtMs{0};
    uint64_t activeResultsFrameId{0};

    struct FrameCycle
    {
        cv::Mat image;
        cv::Mat binary;
        uint64_t frameId{0};
        std::int64_t timestampMs{0};
        bool lanesUpdated{false};
        bool receivedNewAiResult{false};
        bool manualBeforeFsm{false};
        bool aiStale{false};
        bool startupGateReleased{false};
        bool emergencyStopRequested{false};
        bool centerUpdated{false};
        bool frameAvailable{false};
        bool cameraTimedOut{false};
        bool cameraReady{true};
        bool exitRequested{false};
    };

    static void callbackMouse(int event, int x, int y, int flags, void *userdata);
    void handleMouse(int event, int x, int y, int flags);
    void runModel();
    bool scanFreshAiAlerts();
    void advanceAlertTimers(bool decelEvidenceRefreshed);
    void updateAlerts();
    void runFsm(cv::Mat &img);
    bool acquireFrame(FrameCycle &frame);
    void preprocessFrame(FrameCycle &frame);
    void consumeAiSnapshot(FrameCycle &frame);
    void updateSafetyState(FrameCycle &frame);
    void runStateMachines(FrameCycle &frame);
    void calculateControl(FrameCycle &frame);
    void applyFinalStopArbitration(FrameCycle &frame);
    void publishTelemetry(const FrameCycle &frame);
    void sendVehicleCommand(FrameCycle &frame);
    void requestShutdown();

public:
    Icar();
    ~Icar();
    void running();
    bool shouldShutdown() const { return shutdownRequested.load(); }
};
