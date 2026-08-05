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
 * @file busy.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"
#include "fsm/busy_exit_state.hpp"

/**
 * @brief 避障（施工区）控制
 *
 */
class FsmBusy : public FSMState
{
public:
    FsmBusy(std::shared_ptr<Params> par);
    ~FsmBusy();
    void run(Mat &img);
    void show(Mat &img);
    FsmMode getMode();
    void resetLap();
    bool slowing = false; // 减速使能

    // 手动接管相关方法
    void startManualTakeover();
    void endManualTakeover();
    bool isInManualTakeover() { return manualTakeover; }

private:
    bool enable = false;            // 场景检测使能标志
    bool manualTakeover = false;    // 手动接管标志
    bool waitingForTakeover = true; // 等待手动接管标志
    int timeout = 0;
    control_algorithms::BusyConfirmationState busyConfirmation;
    bool drivingThrough = false;    // 退出手动接管后继续在施工区模式行驶，等待左拐退出
    BusyExitState exitState;
    int stationExitCooldown = 0;    // 第一个框停车后等待检测左转的冷却（30帧=1秒）

};
