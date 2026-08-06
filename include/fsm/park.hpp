#pragma once
#include <chrono>
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
 * @file park.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车场控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 * 入库：（parkInsideStep）
 *          1：车库识别（初始化）
 *          2：岔路转弯模式设定
 *          3：道闸前停车识别
 *          4：识别入库标志与当前停车情况
 *          5：开始转弯入库
 *          6：入库结束
 * 出库：（parkOutsideStep）
 *          1：车库识别（初始化）
 *          2：倒车出库
 *          3：识别道闸停车
 *          4：出道闸进入岔路转弯模式
 *          5：退出转弯模式并退出出库阶段
 */

#include "fsm/fsm.hpp"
#include "fsm/park/park_observation.hpp"
#include "fsm/park/park_path_planner.hpp"
#include "fsm/park/park_debug_renderer.hpp"

/**
 * @brief 停车场控制
 *
 */
class FsmPark : public FSMState
{
public:
  FsmPark(std::shared_ptr<Params> par);
  ~FsmPark();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();
  void resetLap();

  bool stopping = false; // 停车等待标志
  bool completedThisLap = false; // 本圈停车任务已完成锁存（仅resetLap清除）
  float spotUp = 0.49;    // 上停车位距离像素百分比[0,1]
  float spotDown = 0.35;  // 下停车位距离像素百分比[0,1]

private:
  /**
   * @brief 车位信息
   *
   */
  class Spots
  {
  public:
    vector<PredictResult> forks; // 停车位箭头信息（size=0:无停车位，size=1:停车位1/2，size=2:停车位1/2/3/4）
    int counter[4] = {0};        // 车位检测计数器
    bool spotEnable[4] = {true}; // 停车位使能标志
    int countRes = 0;            // 车位检测计数器
    bool checked = false;        // 已确定停车位编号标志
    int times = 0;               // 临时计数器

    void reset()
    {
      for (int i = 0; i < 4; i++)
      {
        counter[i] = 0;
        spotEnable[i] = true;
      }
      forks.clear();
      countRes = 0; // 车位检测计数器
      checked = false;
      times = 0;
    }
  };

  /**
   * @brief 停车步骤
   *
   */
  enum Step
  {
    NONE = 0, // 未知状态
    ENABLE,   // 停车场使能
    FORKIN,   // 入库岔路转向
    TRACKIN,  // 入库巡线
    ENTER,    // 入库
    PARKING,  // 停车
    WAIT_PICKUP, // 乘客上车等待
    EXIT,     // 出库
    TRACKOUT, // 出库巡线
    FORKOUT,  // 出库岔路转向
    TARGET_LOST_STOP,    // 目标车位丢失：安全停车，禁止跳过任务
    ENTER_UNCONFIRMED_STOP, // 入库未确认：安全停车，禁止视为停车成功
    EXIT_UNCONFIRMED_STOP, // 出口未确认：安全停车，禁止完成任务
  };
  struct ParkStateData
  {
    enum class ParkMissionProgress
    {
      NOT_STARTED,
      ENTERING,
      PARKED,
      PICKUP_COMPLETED,
      EXIT_REPLAY_COMPLETED
    };

    Step stage = Step::NONE;
    ParkMissionProgress missionProgress{
        ParkMissionProgress::NOT_STARTED};
    int parkMarkerConfirmations = 0;      // NONE：停车场标志连续确认帧数（仅fresh推进）
    int markerMissingFreshFrames = 0;     // 当前阶段AI标志连续缺失帧数（仅fresh推进）
    int choiceApproachConfirmations = 0;  // ENABLE：车位选择标志接近确认帧数
    int gateConfirmations = 0;            // FORKIN/TRACKIN/TRACKOUT：道闸/出口标志确认帧数
    int exitSignSeenConfirmations = 0;    // FORKOUT：出口左转标志连续确认帧数（先武装）
    int exitSignMissingConfirmations = 0; // FORKOUT：出口左转标志连续消失确认帧数
    bool exitSignArmed = false;           // FORKOUT：已连续确认出口标志后才允许缺失计数
    int gateApproachConfirmations = 0;    // TRACKIN（parkSpot==0穿过模式）：道闸接近连续确认
    int enterConfirmations = 0;           // ENTER：真实入库连续确认（lane+path+无PLANNER锁存）
    int enterRecoveryConfirmations = 0;   // ENTER_UNCONFIRMED_STOP：恢复用连续车道确认
    int stageControlFrames = 0;           // 通用阶段超时计数
    int trackInControlFrames = 0;         // TRACKIN 纯控制周期计数（非AI证据）
    std::chrono::steady_clock::time_point forkOutStartedAt;
    std::chrono::steady_clock::time_point stageStartedAt;
    std::chrono::steady_clock::time_point waitStartedAt;
  } state;
  Spots spots;
  bool waitTimerActive = false;
  bool waiting = false;                                           // 停车等待使能
  int countIn = 0;                                                // 入库矫正计数器
  vector<vector<PointX>> pointsEdgeLeftPast, pointsEdgeRightPast; // 记录赛道入库车道线
  ParkDebugRenderer debugRenderer;

  void setStep(Step st);
  void dispatchStage(const ParkObservation &observation,
                     std::chrono::steady_clock::time_point now);
  void handleNone(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleEnable(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleForkIn(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleTrackIn(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleEnter(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleParking(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleWaitPickup(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleExit(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleTrackOut(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleForkOut(const ParkObservation &, std::chrono::steady_clock::time_point);
  void handleTargetLostStop(const ParkObservation &,
                            std::chrono::steady_clock::time_point);
  void handleEnterUnconfirmedStop(const ParkObservation &,
                                  std::chrono::steady_clock::time_point);
  void handleExitUnconfirmedStop(const ParkObservation &,
                                 std::chrono::steady_clock::time_point);
  void updateGeometryPolicy();
  bool canTransition(Step from, Step to) const;
  void updateMissionProgressForTransition(Step from, Step to);
  bool gateRequiresStop(const ParkObservation &observation) const;
  void reset();
  void replanTracking();
  void replanTracking(bool left);
  vector<PredictResult> findParkStation(const vector<PredictResult> &results) const;

  friend struct ParkFsmTestFixture;
};
