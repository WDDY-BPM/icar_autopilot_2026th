#pragma once
#include <atomic>
#include <condition_variable>
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
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    std::thread captureThread;
    std::atomic<bool> captureRunning{false};
    std::mutex latestCaptureMutex;
    std::condition_variable latestCaptureCv;
    cv::Mat latestCaptureFrame;
    uint64_t latestCaptureSequence{0};
    uint64_t consumedCaptureSequence{0};
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
    int cameraFreshFrames = 0;

    bool updateStartupGate(bool receivedNewAiResult)
    {
        if (!startupEnvironmentChecked)
        {
            startupEnvironmentChecked = true;
            const char *preconfirmed = std::getenv("ICAR_START_CONE_PRECONFIRMED");
            if (params->config.requireStartCone && preconfirmed &&
                std::string(preconfirmed) == "1")
            {
                startupGateState = StartupGateState::WAIT_FOR_REMOVAL;
                startupConeSeenCount = 3;
                startupConeMissingCount = 0;
                std::cout << "[Startup] Cone preconfirmed by launcher. "
                             "Waiting for removal." << std::endl;
            }
        }
        if (startupGateState == StartupGateState::RELEASED)
            return true;

        bool coneDetected = startupConeDetected;
        if (receivedNewAiResult)
        {
            coneDetected = false;
            for (const auto &result : params->results)
            {
                if (result.type == LABEL_CONE && result.width >= 10 && result.height >= 10)
                {
                    coneDetected = true;
                    break;
                }
            }
            startupConeDetected = coneDetected;
        }

        const auto &laneQuality = params->track->quality;
        // At a curved start line, temporal center jump and perspective lane
        // width variation can legitimately exceed the straight-road quality
        // thresholds.  Requiring those metrics here could keep the startup
        // gate closed forever even though both physical lane edges are sound.
        // The normal controller performs its own recovery checks after release.
        const bool laneValid = laneQuality.leftReliable &&
            laneQuality.rightReliable && laneQuality.coversBottom &&
            laneQuality.commonRows >= 20;
        startupLaneValidCount = laneValid ? startupLaneValidCount + 1 : 0;

        if (!params->config.requireStartCone)
        {
            if (startupLaneValidCount >= params->config.startupStableFrames)
            {
                startupGateState = StartupGateState::RELEASED;
                params->ctrl.countAcc = 0;
                params->ctrl.startupSteeringCount = 0;
                params->setStopReason(control_algorithms::StopReason::STARTUP, false);
                std::cout << "[Startup] Cone gate disabled; stable lane confirmed. AUTO released." << std::endl;
                return true;
            }
            params->setStopReason(control_algorithms::StopReason::STARTUP, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            return false;
        }

        if (startupGateState == StartupGateState::WAIT_FOR_CONE)
        {
            if (receivedNewAiResult)
                startupConeSeenCount = coneDetected ? startupConeSeenCount + 1 : 0;

            if (startupConeSeenCount >= 3)
            {
                startupGateState = StartupGateState::WAIT_FOR_REMOVAL;
                startupConeMissingCount = 0;
                std::cout << "[Startup] Cone confirmed. Remove it to start." << std::endl;
            }
        }
        else if (receivedNewAiResult)
        {
            startupConeMissingCount = coneDetected ? 0 : startupConeMissingCount + 1;
            if (startupConeMissingCount >= 5 && startupLaneValidCount >= params->config.startupStableFrames)
            {
                startupGateState = StartupGateState::RELEASED;
                params->ctrl.countAcc = 0;
                params->ctrl.startupSteeringCount = 0;
                params->setStopReason(control_algorithms::StopReason::STARTUP, false);
                std::cout << "[Startup] Cone removed and lane stable. AUTO released." << std::endl;
                return true;
            }
        }

        params->setStopReason(control_algorithms::StopReason::STARTUP, true);
        params->ctrl.speed = 0.0f;
        params->ctrl.servo = PWMSERVOMID;
        if (++startupDiagnosticFrames % 30 == 0)
        {
            const char *state = startupGateState == StartupGateState::WAIT_FOR_CONE
                ? "WAIT_FOR_CONE" : "WAIT_FOR_REMOVAL";
            std::cout << "[Startup] state=" << state
                      << " coneDetected=" << coneDetected
                      << " coneSeen=" << startupConeSeenCount
                      << " coneMissing=" << startupConeMissingCount
                      << " leftReliable=" << laneQuality.leftReliable
                      << " rightReliable=" << laneQuality.rightReliable
                      << " coversBottom=" << laneQuality.coversBottom
                      << " commonRows=" << laneQuality.commonRows
                      << " laneValid=" << laneValid
                      << " laneFrames=" << startupLaneValidCount
                      << " confidence=" << laneQuality.confidence
                      << " centerJump=" << laneQuality.centerJump
                      << " widthVariation=" << laneQuality.widthVariation
                      << std::endl;
        }
        return false;
    }
    cv::Mat imgShare;
    uint64_t imgShareFrameId{0};
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};
    std::atomic<bool> shuttingDown{false};
    std::mutex mtxRes;
    std::atomic<bool> readyRes{false};
    std::vector<PredictResult> latestResults; // AI线程单独写，主线程按帧复制
    uint64_t latestResultsFrameId{0};
    uint64_t activeResultsFrameId{0};

    /**
     * @brief 鼠标的事件回调函数
     *
     */
    static void callbackMouse(int event, int x, int y, int flags, void *userdata)
    {
        Icar *self = static_cast<Icar *>(userdata);
        if (self)
            self->handleMouse(event, x, y, flags);
    }
    void handleMouse(int event, int x, int y, int flags)
    {
        double value;
        switch (event)
        {
        case cv::EVENT_MOUSEWHEEL: // 鼠标滑球
        {
            value = cv::getMouseWheelDelta(flags); // 获取滑球滚动值
            if (value > 0)
                show->index++;
            else if (value < 0)
                show->index--;

            if (show->index < 0)
                show->index = 0;
            if (show->index > show->frameMax)
                if (show->index > show->frameMax)
                    show->index = show->frameMax;
            break;
        }
        default:
            break;
        }
    }

    /**
     * @brief AI 模型推理
     *
     */
    void runModel()
    {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this]
                   { return readyImg.load() || shuttingDown.load(); });
        if (shuttingDown)
            return;
        cv::Mat img = imgShare.clone(); // 图像拷贝出来再释放锁
        const uint64_t inferenceFrameId = imgShareFrameId;
        readyImg = false;
        lock.unlock();

        // 手动接管期间跳过AI推理
        if (!params->manualTakeover)
        {
            // 启动AI推理
            detection->inference(img);
            std::lock_guard<std::mutex> lock_result(mtxRes);
            latestResults = detection->results;
            latestResultsFrameId = inferenceFrameId;
            readyRes = true;
        }
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        // 圈数变更时复位所有FSM状态
        if (lastLap != params->currentLap)
        {
            lastLap = params->currentLap;
            fsmFactory.stop->resetLap();
            fsmFactory.park->resetLap();
            fsmFactory.fork->resetLap();
            fsmFactory.yfork->resetLap();
            fsmFactory.slow->resetLap();
            fsmFactory.busy->resetLap();
            fsmFactory.station->resetLap();
            fsmFactory.obstacle->resetLap();
            params->alertCountdown = 0;                // 复位蜂鸣器报警
            params->alertDecelCount = 0;               // 复位报警减速
            params->busyAlertCountdown = 0;            // 复位施工区蜂鸣
        }

        if (params->mode == FsmMode::STATION ||
            params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW ||
            params->mode == FsmMode::YFORK) // 状态复位
            params->mode = FsmMode::NORMAL;

        // 处理手动接管（优先执行，跳过所有FSM检测）
        // Monitoring connection does not trigger takeover.

        params->manualTakeover = fsmFactory.busy->isInManualTakeover();
        if (params->manualTakeover)
        {
            cout << "[Icar] Manual takeover active." << endl;

            // 检查是否返回自动模式
            if (fsmFactory.manual->checkForReturnKey())
            {
                fsmFactory.busy->endManualTakeover();
                params->setStopReason(control_algorithms::StopReason::MANUAL, false);
                params->mode = FsmMode::NORMAL;   // 恢复自动模式
                params->takeoverJustEnded = true; // 通知各FSM手动接管刚结束
                params->autoRecoveryFrames = 2;
                params->setStopReason(control_algorithms::StopReason::STARTUP, true);
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
            }
            else if (fsmFactory.manual->isConnected() &&
                     fsmFactory.manual->isManualControl())
            {
                fsmFactory.manual->applyManualControl(&params->ctrl.speed, &params->ctrl.servo);
                params->setStopReason(control_algorithms::StopReason::MANUAL, false);
            }
            else
            {
                params->setStopReason(control_algorithms::StopReason::MANUAL, true);
                params->ctrl.speed = 0;
                params->ctrl.servo = PWMSERVOMID;
            }

            if (params->mode != params->modeLast)
            {
                client->buzzerSound(client->BUZZER_DING); // 提示音效
                params->modeLast = params->mode;
            }
            return; // 手动接管期间跳过所有FSM检测
        }

        // 根据当前圈配置设置功能使能（覆盖全局配置）
        params->config.fork = params->config.currentLapConfig->fork;
        params->config.park = params->config.currentLapConfig->park;
        params->config.busy = params->config.currentLapConfig->busy;
        params->config.slow = params->config.currentLapConfig->slow;
        params->config.stop = params->config.currentLapConfig->stop;
        params->config.cross = params->config.currentLapConfig->cross;
        params->config.yfork = params->config.currentLapConfig->yfork;
        params->config.station = params->config.currentLapConfig->station;
        params->config.obstacle = params->config.currentLapConfig->obstacle;

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        params->mode = fsmFactory.cross->getMode();

        // ===== 路线隔离：仅在当前圈使能时调用对应FSM =====
        if (params->config.park)
        {
            if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK || params->mode == FsmMode::CROSS)
            {
                fsmFactory.park->run(img);
                FsmMode mode = fsmFactory.park->getMode();
                if (mode != FsmMode::NORMAL)
                    params->mode = mode;
            }
        }
        if (params->config.fork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.fork->run(img);
                params->mode = fsmFactory.fork->getMode();
            }
        }
        if (params->config.yfork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.yfork->run(img);
                params->mode = fsmFactory.yfork->getMode();
            }
        }
        if (params->config.slow)
        {
            fsmFactory.slow->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.slow->getMode();
        }
        if (params->config.busy)
        {
            fsmFactory.busy->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.busy->getMode();
            // 施工区鸣笛（每秒3次，间隔10帧≈333ms）
            if (params->busyAlertCountdown > 0)
            {
                if (params->busyAlertCountdown % 10 == 0)
                    client->buzzerSound(client->BUZZER_WARNNING);
                params->busyAlertCountdown--;
            }
        }
        if (params->config.station)
        {
            fsmFactory.station->run(img);
            FsmMode stationMode = fsmFactory.station->getMode();
            // 不覆盖YFORK模式，让yfork状态能正常显示
            if (stationMode != FsmMode::NORMAL && params->mode != FsmMode::YFORK)
                params->mode = stationMode;
        }

        // 全局障碍物检测（锥桶/行人），施工区/停车场/Y型岔路口期间不检测
        params->ctrl.obstacleSlow = false;
        if (params->config.obstacle &&
            params->mode != FsmMode::BUSY &&
            params->mode != FsmMode::PARK &&
            params->mode != FsmMode::YFORK)
            fsmFactory.obstacle->run(img);

        // 蜂鸣器报警目标检测（图像下方4/5区域），遇目标持续鸣笛1s
        if (params->config.alertTarget != "none")
        {
            int alertLabel = -1;
            if (params->config.alertTarget == "cone")
                alertLabel = LABEL_CONE;
            else if (params->config.alertTarget == "person")
                alertLabel = LABEL_PERSON;
            else if (params->config.alertTarget == "busy")
                alertLabel = LABEL_BUSY;
            else if (params->config.alertTarget == "limit")
                alertLabel = LABEL_LIMIT;
            else if (params->config.alertTarget == "unlimit")
                alertLabel = LABEL_UNLIMIT;
            else if (params->config.alertTarget == "park")
                alertLabel = LABEL_PARK;

            // 施工区由FSM自身负责鸣笛，不重复触发
            if (alertLabel == LABEL_BUSY && params->config.currentLapConfig &&
                params->config.currentLapConfig->busy)
                alertLabel = -1;

            if (alertLabel >= 0)
            {
                bool targetFound = false;
                {
                    std::lock_guard<std::mutex> lock(mtxRes);
                    for (auto &r : params->results)
                    {
                        bool posOk = (alertLabel == LABEL_LIMIT || alertLabel == LABEL_PARK)
                                         ? (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2) // 左右两侧1/5
                                         : (r.y + r.height) > ROWSIMAGE * 0.2;                        // 其他：底部4/5区域
                        if (r.type == alertLabel && posOk)
                        {
                            targetFound = true;
                            break;
                        }
                    }
                }
                if (targetFound && params->alertCountdown <= 0)
                    params->alertCountdown = 30; // 启动最少1秒倒计时
            }
            if (params->alertCountdown > 0)
            {
                if (params->alertCountdown % 10 == 0) // 每~333ms鸣笛一次，最少3次
                    client->buzzerSound(client->BUZZER_WARNNING);
                params->alertCountdown--;
            }
        }

        {
            bool decelFound = false;
            {
                std::lock_guard<std::mutex> lock(mtxRes);
                for (auto &r : params->results)
                {
                    bool posOk = false;
                    if (r.type == LABEL_LIMIT || r.type == LABEL_PARK)
                        posOk = (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2); // 左右两侧1/5
                    else if (r.type == LABEL_CONE || r.type == LABEL_PERSON || r.type == LABEL_UNLIMIT)
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2; // 底部4/5区域
                    else if (r.type == LABEL_BUSY)
                    {
                        if (params->config.currentLapConfig && params->config.currentLapConfig->busy)
                            continue;
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2;
                    }
                    else
                        continue;

                    if (posOk)
                    {
                        decelFound = true;
                        break;
                    }
                }
            }
            if (decelFound)
                params->alertDecelCount = 5;
            else if (params->alertDecelCount > 0)
                params->alertDecelCount--;
        }

        if (params->mode != params->modeLast && params->alertCountdown <= 0)
        {
            client->buzzerSound(client->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }

public:
    /**
     * @brief 参数初始化
     *
     */
    Icar()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        motion = make_shared<Motion>();                        // 运动控制器
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化TCP通信客户端
        client = make_shared<Client>();
        if (!client->start())
        {
            printf("[Error]: Socket init failed!!!\n");
            exit(-1);
        }
        client->buzzerSound(client->BUZZER_OK); // 提示音效

        // 相机初始化
        // USB摄像头初始化
        if (params->config.debug)
            capture = make_shared<cv::VideoCapture>(params->config.video); // 打开本地视频
        else
        {
            capture = make_shared<cv::VideoCapture>("/dev/video0", cv::CAP_V4L2);
            if (!capture->isOpened())
                capture = make_shared<cv::VideoCapture>("/dev/video0");
        }
        if (!capture->isOpened())
        {
            printf("[Error]: Can not open video device!!!\n");
            exit(-1);
        }
        capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        capture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率
        if (!params->config.debug)
            capture->set(cv::CAP_PROP_BUFFERSIZE, 1);

        if (params->config.debug)
        {
            show = make_shared<Show>(4); // 调试UI初始化
            show->frameMax = capture->get(cv::CAP_PROP_FRAME_COUNT) - 1;
            cv::createTrackbar("Frame", "ICAR", &show->index, show->frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
            cv::setMouseCallback("ICAR", this->callbackMouse);                                     // 创建鼠标键盘快捷键事件
        }
        else
        {
            std::cout << "[Camera] Latest-frame capture enabled; backend="
                      << capture->get(cv::CAP_PROP_BACKEND) << std::endl;
            captureRunning = true;
            captureThread = std::thread([this]() {
                while (captureRunning)
                {
                    cv::Mat frame;
                    if (!capture->read(frame))
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    {
                        std::lock_guard<std::mutex> lock(latestCaptureMutex);
                        latestCaptureFrame = std::move(frame);
                        ++latestCaptureSequence;
                    }
                    latestCaptureCv.notify_one();
                }
            });
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);         // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);         // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);         // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params);       // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);         // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);         // 慢行区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 全局障碍物检测实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params);       // Y型岔路口控制实例化
        fsmFactory.station = make_shared<FsmStation>(params);   // 停靠站控制实例化

        // 手动控制线程初始化
        fsmFactory.manual = make_shared<ManualControlThread>();

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Icar::runModel, this));
        loops->start(); // RL开始推理

        // 启动手动控制线程
        fsmFactory.manual->start();

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Icar()
    {
        captureRunning = false;
        latestCaptureCv.notify_all();
        if (captureThread.joinable())
            captureThread.join();
        if (fsmFactory.manual)
            fsmFactory.manual->stop();
        shuttingDown = true;
        cvImg.notify_all();
        if (loops)
            loops->shutdown();
    };

    /**
     * @brief 程序主循环
     *
     */
    void running()
    {
        //[01] 视频源读取
        cv::Mat img;
        if (params->config.debug) // 综合显示调试UI窗口
        {
            if (show->indexLast == show->index) // 图像帧未更新
            {
                if (client->keypress.exchange(false))
                {
                    client->buzzerSound(client->BUZZER_FINISH); // 祖传提示音效
                    printf("-----> System Exit!!! <-----\n");
                    client->carControl(0.0f, PWMSERVOMID);
                    exit(0); // 调试退出前先停车
                }
                show->show();        // 显示综合绘图
                client->sendHeart(); // 发送给服务器在线心跳
                usleep(10 * 1000);   // us延迟
                return;
            }

            capture->set(cv::CAP_PROP_POS_FRAMES, show->index); // 设置读取帧
            if (!capture->read(img))
            {
                params->setStopReason(control_algorithms::StopReason::CAMERA, true);
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
                previousFinalServo = PWMSERVOMID;
                motion->reset();
                client->carControl(0.0f, PWMSERVOMID);
                return;
            }
            show->indexLast = show->index;
        }
        else
        {
            std::unique_lock<std::mutex> lock(latestCaptureMutex);
            const bool frameReady = latestCaptureCv.wait_for(
                lock, std::chrono::milliseconds(150), [this]() {
                    return !captureRunning ||
                        latestCaptureSequence != consumedCaptureSequence;
                });
            if (frameReady && latestCaptureSequence != consumedCaptureSequence)
            {
                img = latestCaptureFrame.clone();
                consumedCaptureSequence = latestCaptureSequence;
            }
            lock.unlock();
            if (img.empty())
            {
                if (!params->hasStopReason(control_algorithms::StopReason::CAMERA))
                    std::cout << "[Camera] Frame timeout; CAMERA stop active." << std::endl;
                params->setStopReason(control_algorithms::StopReason::CAMERA, true);
                cameraFreshFrames = 0;
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
                previousFinalServo = PWMSERVOMID;
                motion->reset();
                client->carControl(0.0f, PWMSERVOMID);
                return;
            }
        }
        if (params->hasStopReason(control_algorithms::StopReason::CAMERA))
        {
            if (++cameraFreshFrames >= 3)
            {
                params->setStopReason(control_algorithms::StopReason::CAMERA, false);
                cameraFreshFrames = 0;
                std::cout << "[Camera] Fresh frames recovered; CAMERA stop cleared." << std::endl;
            }
        }

        uint64_t currentFrameId = 0;
        int64_t currentFrameTimestampMs = 0;

        //[02] 图像存储
        if (params->config.saveImg && !params->config.debug) // 存储原始图像
            savePicture(img);
        else if (params->config.saveImg && params->config.debug) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        cv::Mat imgBin;
        predeal->correction(img); // 图像矫正
        // Publish an unannotated, geometrically corrected frame before lane,
        // AI and FSM processing. Overlay coordinates therefore match exactly.
        if (fsmFactory.manual->isConnected())
            currentFrameId = fsmFactory.manual->sendImage(
                img, &currentFrameTimestampMs);
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        {
            std::lock_guard<std::mutex> lock(mtxImg);
            imgShare = img.clone();
            imgShareFrameId = currentFrameId;
            readyImg = true;
        }
        cvImg.notify_one();
        /*-------------------------------------------------------*/
        imgBin = predeal->binaryzation(img); // 图像二值化
        //[04] Track recognition (skipped during manual takeover).
        const bool lanesUpdatedThisFrame =
            !fsmFactory.busy->isInManualTakeover();
        if (lanesUpdatedThisFrame)
        {
            const auto containsFork = [](const std::vector<PredictResult> &results) {
                return std::any_of(results.begin(), results.end(),
                    [](const PredictResult &result) { return result.type == LABEL_FORK; });
            };
            bool forkMarkerActive = containsFork(params->results);
            {
                std::lock_guard<std::mutex> resultLock(mtxRes);
                if (readyRes)
                    forkMarkerActive = containsFork(latestResults);
            }
            params->track->allowOuterEnvelope = !forkMarkerActive;
            params->track->handle(imgBin);
            const bool widthLearningMode = params->track->quality.valid &&
                !params->manualTakeover && !params->ctrl.fitting &&
                !forkMarkerActive &&
                (params->mode == FsmMode::NORMAL || params->mode == FsmMode::CURVE);
            center->observeLaneWidth(params->track->pointsEdgeLeft,
                                     params->track->pointsEdgeRight,
                                     widthLearningMode);
        }
        if (params->config.debug)
        {
            show->setNewWindow(1, "Bin", imgBin);
            cv::Mat imgTrack = img.clone();
            params->track->drawImage(imgTrack); // 图像绘制赛道识别结果
            show->setNewWindow(2, "Track", imgTrack);
            if (params->config.saveIpm && params->config.saveImg)
            {
                cv::Mat imgIpm;
                ipm.homography(imgTrack, imgIpm);
                savePicture(imgIpm); // 保存图像
            }
        }

        // 在主线程按帧获取AI结果快照；FSM不再与推理线程共享可变vector
        bool receivedNewAiResult = false;
        {
            std::lock_guard<std::mutex> resultLock(mtxRes);
            if (readyRes)
            {
                params->results = latestResults;
                activeResultsFrameId = latestResultsFrameId;
                readyRes = false;
                receivedNewAiResult = true;
            }
        }

        params->aiResultFresh = receivedNewAiResult;

        const bool startupGateReleased = updateStartupGate(receivedNewAiResult);

        //[05] 有限状态机任务执行。锁存急停时不得推进任何有状态 FSM。
        params->ctrl.fitting = false;
        const bool emergencyStopRequested = fsmFactory.manual->isEmergencyStopRequested();
        params->setStopReason(control_algorithms::StopReason::EMERGENCY,
                              emergencyStopRequested);
        if (emergencyStopRequested && !emergencyStopWasActive)
        {
            emergencyStopWasActive = true;
        }
        else if (!emergencyStopRequested && emergencyStopWasActive)
        {
            // All FSM state and parking trajectory history were frozen with the vehicle.
            // Keep them intact, gather fresh lane data while stopped, then continue.
            params->autoRecoveryFrames = 15;
            emergencyStopWasActive = false;
        }

        if (startupGateReleased && !emergencyStopRequested &&
            params->autoRecoveryFrames <= 0 && !params->laneSafetyStop)
            runFsm(imgBin);

        // 同步手动接管状态（runFsm中endManualTakeover可能改变了状态，但params->manualTakeover未更新）
        params->manualTakeover = fsmFactory.busy->isInManualTakeover();

        // A remote STOP is latched and has priority in AUTO and MANUAL modes.
        if (emergencyStopRequested)
        {
            params->setStopReason(control_algorithms::StopReason::EMERGENCY, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }


        //[06] Calculate the lane control center in autonomous mode.
        bool centerUpdatedThisFrame = false;
        if (startupGateReleased && !emergencyStopRequested &&
            !params->manualTakeover && params->autoRecoveryFrames <= 0)
        {
            center->fitting(params);
            centerUpdatedThisFrame = true;
            const int laneUnconfirmedFrames = control_algorithms::updateLaneUnconfirmed(
                laneUnconfirmedState, center->controlValid, 5);
            const bool safetyLaneMode = params->mode == FsmMode::NORMAL ||
                                        params->mode == FsmMode::CURVE ||
                                        params->mode == FsmMode::CROSS ||
                                        params->mode == FsmMode::STOP ||
                                        params->mode == FsmMode::SLOW ||
                                        params->mode == FsmMode::STATION;
            params->laneSafetyStop = control_algorithms::updateLaneSafetyStop(
                params->laneSafetyStop, safetyLaneMode, center->controlValid,
                center->laneInvalidFrames, center->laneRecoveryFrames, 7, 5,
                laneUnconfirmedFrames, 30);
            params->setStopReason(control_algorithms::StopReason::LANE,
                                  params->laneSafetyStop);
        }

        params->ctrl.stop = params->mustStop();

        //[07] 车辆运动控制（仅手动接管时跳过）
        static auto lastSteeringUpdate = std::chrono::steady_clock::now();
        static bool steeringClockInitialized = false;
        const auto steeringNow = std::chrono::steady_clock::now();
        float steeringDt = 1.0f / 30.0f;
        if (steeringClockInitialized)
            steeringDt = std::chrono::duration<float>(steeringNow - lastSteeringUpdate).count();
        lastSteeringUpdate = steeringNow;
        steeringClockInitialized = true;

        const bool automaticControlActive =
            startupGateReleased && !emergencyStopRequested &&
            !params->manualTakeover && params->autoRecoveryFrames <= 0;
        const bool laneHold = params->laneSafetyStop;
        static bool laneHoldWasActive = false;
        if (automaticControlActive && !laneHold)
        {
            motion->poseControl(params, steeringDt);
            motion->speedControl(params);

            const bool strictLaneMode = params->mode == FsmMode::NORMAL ||
                                        params->mode == FsmMode::CURVE ||
                                        params->mode == FsmMode::CROSS ||
                                        params->mode == FsmMode::STOP ||
                                        params->mode == FsmMode::SLOW ||
                                        params->mode == FsmMode::STATION;
            const bool lowConfidenceLane =
                center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID ||
                center->recoveryMode == LaneRecoveryMode::LEFT_SINGLE ||
                center->recoveryMode == LaneRecoveryMode::RIGHT_SINGLE;
            if (lowConfidenceLane)
                params->ctrl.speed = std::min(params->ctrl.speed, 0.15f);
            else if (center->recoveryMode == LaneRecoveryMode::RELAXED_DUAL)
                params->ctrl.speed = std::min(params->ctrl.speed,
                    std::min(params->config.velCurve, 0.20f));

            if (strictLaneMode && laneUnconfirmedState.frames > 0)
            {
                params->ctrl.speed = std::min(params->ctrl.speed, 0.10f);
                params->ctrl.servo = motion->syncServoCommand(previousFinalServo);
            }
        }
        else if (automaticControlActive && laneHold)
        {
            if (!laneHoldWasActive)
                params->ctrl.countAcc = 0;
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = motion->limitServoCommand(
                PWMSERVOMID, steeringDt, 250.0f);
            if (std::abs(static_cast<int>(params->ctrl.servo) - PWMSERVOMID) <= 3)
                motion->resetControl();
        }
        else if (params->manualTakeover && !emergencyStopRequested)
        {
            params->laneSafetyStop = false;
            params->setStopReason(control_algorithms::StopReason::LANE, false);
            motion->resetControl();
            params->ctrl.servo = motion->limitServoCommand(
                params->ctrl.servo, steeringDt, params->config.servoRate);
        }
        else
        {
            // Startup gating and emergency/recovery stops must return to center
            // immediately instead of passing through the normal slew limiter.
            motion->reset();
        }
        laneHoldWasActive = automaticControlActive && laneHold;
        if (!emergencyStopRequested && params->autoRecoveryFrames > 0)
        {
            params->setStopReason(control_algorithms::StopReason::STARTUP, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            params->autoRecoveryFrames--;
            if (params->autoRecoveryFrames == 0)
                params->setStopReason(control_algorithms::StopReason::STARTUP, false);
        }

        // Reassert after all recovery/control logic so the latch cannot be cleared this frame.
        if (emergencyStopRequested)
        {
            params->setStopReason(control_algorithms::StopReason::EMERGENCY, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
        if (!startupGateReleased)
        {
            params->setStopReason(control_algorithms::StopReason::STARTUP, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }

        params->ctrl.stop = params->mustStop();
        if (params->ctrl.stop)
        {
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
        previousFinalServo = params->ctrl.servo;

        // Publish the final command after automatic/manual limiting and all
        // emergency/startup overrides, so telemetry is not one frame stale.
        fsmFactory.manual->updateVehicleState(
            params->ctrl.speed, params->ctrl.servo, params->manualTakeover);
        // Construct and publish overlays at no more than 12.5 Hz. This avoids
        // allocating and serializing JSON on every 30 Hz control iteration.
        const auto overlayNow = std::chrono::steady_clock::now();
        if (fsmFactory.manual->isConnected() && currentFrameId != 0 &&
            overlayNow - lastOverlayBuilt >= std::chrono::milliseconds(80))
        {
            lastOverlayBuilt = overlayNow;
            const bool leftOverlayValid = lanesUpdatedThisFrame &&
                                          (params->track->quality.leftReliable ||
                                           center->singleSide == -1 ||
                                           center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID) &&
                                          !params->manualTakeover;
            const bool rightOverlayValid = lanesUpdatedThisFrame &&
                                           (params->track->quality.rightReliable ||
                                            center->singleSide == 1 ||
                                            center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID) &&
                                           !params->manualTakeover;
            const bool lanesValid = leftOverlayValid || rightOverlayValid;
            const bool centerValid = centerUpdatedThisFrame &&
                                     center->controlValid &&
                                     params->ctrl.centerEdge.size() >= 12 &&
                                     !params->manualTakeover;
            nlohmann::json overlay;
            overlay["frame_id"] = currentFrameId;
            overlay["frame_timestamp_ms"] = currentFrameTimestampMs;
            overlay["timestamp_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            overlay["mode"] = static_cast<int>(params->mode);
            overlay["speed"] = params->ctrl.speed;
            overlay["steering"] = params->ctrl.servo;
            overlay["center"] = params->ctrl.center;
            overlay["center_error"] = params->ctrl.center - COLSIMAGE / 2;
            const auto recoveryModeName = [](LaneRecoveryMode mode) {
                switch (mode) {
                case LaneRecoveryMode::STRICT_DUAL: return "STRICT_DUAL";
                case LaneRecoveryMode::RELAXED_DUAL: return "RELAXED_DUAL";
                case LaneRecoveryMode::WEAK_HYBRID: return "WEAK_HYBRID";
                case LaneRecoveryMode::LEFT_SINGLE: return "LEFT_SINGLE";
                case LaneRecoveryMode::RIGHT_SINGLE: return "RIGHT_SINGLE";
                default: return "INVALID";
                }
            };
            overlay["recovery_mode"] = recoveryModeName(center->recoveryMode);
            overlay["ctrl_stop"] = params->ctrl.stop;
            overlay["must_stop"] = params->mustStop();
            overlay["stop_reasons"] = params->stopReasonString();
            overlay["camera_stop"] = params->hasStopReason(
                control_algorithms::StopReason::CAMERA);
            overlay["lane_safety_stop"] = params->laneSafetyStop;
            overlay["lanes_valid"] = lanesValid;
            overlay["lanes_frame_id"] = lanesValid ? currentFrameId : 0;
            overlay["center_valid"] = centerValid;
            overlay["center_frame_id"] = centerValid ? currentFrameId : 0;
            overlay["edge"] = {
                {"left_count", leftOverlayValid ? params->track->pointsEdgeLeft.size() : 0},
                {"right_count", rightOverlayValid ? params->track->pointsEdgeRight.size() : 0},
                {"valid_left", leftOverlayValid ? center->validRowsLeft : 0},
                {"valid_right", rightOverlayValid ? center->validRowsRight : 0},
                {"sigma_center", centerValid ? center->sigmaCenter : 0.0},
                {"line_area", centerValid ? params->ctrl.lineArea : 0},
                {"near_center", center->nearCenterValid ? center->nearCenter : -1},
                {"far_center", center->farCenterValid ? center->farCenter : -1},
                {"near_error", center->nearCenterValid
                    ? center->nearCenter - COLSIMAGE / 2 : 0},
                {"far_error", center->farCenterValid
                    ? center->farCenter - COLSIMAGE / 2 : 0},
                {"heading_error", center->headingError},
                {"heading_correction", center->headingCorrection},
                {"heading_confidence", center->headingConfidence},
                {"near_samples", center->nearCenterSamples},
                {"far_samples", center->farCenterSamples},
                {"lane_confidence", params->track->quality.confidence},
                {"common_rows", params->track->quality.commonRows},
                {"invalid_frames", center->laneInvalidFrames},
                {"recovery_frames", center->laneRecoveryFrames},
                {"unconfirmed_frames", laneUnconfirmedState.frames},
                {"left_reliable", params->track->quality.leftReliable},
                {"right_reliable", params->track->quality.rightReliable},
                {"left_strict", params->track->quality.leftReliable},
                {"right_strict", params->track->quality.rightReliable},
                {"left_single_usable", params->track->quality.leftSingleUsable},
                {"right_single_usable", params->track->quality.rightSingleUsable},
                {"left_interior_points", params->track->quality.leftInteriorPoints},
                {"right_interior_points", params->track->quality.rightInteriorPoints},
                {"left_border_ratio", params->track->quality.leftBorderRatio},
                {"right_border_ratio", params->track->quality.rightBorderRatio},
                {"lane_width_ready", center->laneWidthProfileReady()},
                {"usable_center_rows", center->usableCenterRows},
                {"weak_hybrid_active", center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID},
                {"strict_dual", center->recoveryMode == LaneRecoveryMode::STRICT_DUAL},
                {"relaxed_dual", center->recoveryMode == LaneRecoveryMode::RELAXED_DUAL},
                {"selected_recovery_side", center->selectedRecoverySide},
                {"single_side", center->singleSide},
                {"raw_center_jump", center->rawCenterJump},
                {"applied_center_step", center->appliedCenterStep},
                {"control_valid", center->controlValid}
            };

            auto samplePoints = [](const std::vector<PointX> &points) {
                nlohmann::json sampled = nlohmann::json::array();
                constexpr size_t stride = 4;
                for (size_t i = 0; i < points.size(); i += stride)
                    sampled.push_back({points[i].y, points[i].x});
                if (!points.empty() && (points.size() - 1) % stride != 0)
                    sampled.push_back({points.back().y, points.back().x});
                return sampled;
            };
            overlay["left"] = leftOverlayValid
                ? samplePoints(params->track->pointsEdgeLeft)
                : nlohmann::json::array();
            overlay["right"] = rightOverlayValid
                ? samplePoints(params->track->pointsEdgeRight)
                : nlohmann::json::array();
            overlay["center_line"] = centerValid
                ? samplePoints(params->ctrl.centerEdge)
                : nlohmann::json::array();
            overlay["detections"] = nlohmann::json::array();
            overlay["detections_frame_id"] = activeResultsFrameId;
            for (const auto &result : params->results)
            {
                overlay["detections"].push_back({
                    {"type", result.type}, {"label", result.label},
                    {"score", result.score}, {"x", result.x}, {"y", result.y},
                    {"w", result.width}, {"h", result.height}
                });
            }
            fsmFactory.manual->sendOverlay(overlay.dump());
        }
        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img, params->results); // 使用主线程结果快照绘框
            center->drawImage(params, img); // 图像绘制控制路径
            motion->drawImage(params, img); // 图像绘制速度
            show->setNewWindow(3, "Ctrl", img);

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
};
