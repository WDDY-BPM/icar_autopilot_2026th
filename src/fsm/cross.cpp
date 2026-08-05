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
 * @file cross.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 斑马线停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/cross.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmCross::FsmCross(std::shared_ptr<Params> par)
    : FSMState(FsmMode::CROSS, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmCross::~FsmCross()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmCross::getMode()
{
    // 输出场景状态结果
    if (step == Step::NONE || !params->config.currentLapConfig->cross)
        return FsmMode::NORMAL;
    else
        return FsmMode::CROSS;
}

/**
 * @brief Run the lap crossing state machine. Only fresh AI publications can
 * advance appearance, pass, or disappearance confirmation.
 */
void FsmCross::run(Mat &img)
{
    if (!params->config.currentLapConfig->cross)
        return;

    countInit = std::min(countInit + 1, 999);
    if (countInit < 60)
        return;

    bool crossDetected = false;
    bool passCandidate = false;
    if (params->aiResultFresh)
    {
        for (const auto &result : params->results)
        {
            if (result.type != LABEL_CROSS)
                continue;
            crossDetected = true;
            if (result.y + result.height > ROWSIMAGE * 2 / 3)
                passCandidate = true;
        }
    }

    const bool finalLap = params->currentLap >= params->totalLaps;
    const bool lapTaskComplete = !params->lapTaskRequired || params->lapTaskCompleted;
    const auto event = control_algorithms::updateCrossConfirmation(
        confirmation, params->aiResultFresh, crossDetected, passCandidate,
        finalLap, lapTaskComplete);

    if (event == control_algorithms::CrossConfirmationEvent::LAP_PASSED)
    {
        printf("[Cross] Lap %d fully passed\n", params->currentLap);
        params->nextLap();
        setStep(Step::NONE);
        return;
    }
    if (event == control_algorithms::CrossConfirmationEvent::FINAL_STOP)
    {
        printf("[Cross] Final cross fully passed; stopping vehicle\n");
        setStep(Step::STOP);
        return;
    }

    if (step != Step::STOP)
        step = confirmation.linePassed || confirmation.passFrames > 0
            ? Step::ENABLE : Step::NONE;
    if (step == Step::STOP)
        params->setStopReason(control_algorithms::StopReason::CROSS, true);
}
/**
 * @brief Draw cross state diagnostics.
 */
void FsmCross::show(Mat &img)
{
    if (params->mode != FsmMode::CROSS)
        return;

    putText(img, "[8] Cross", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
    if (step == Step::ENABLE)
        putText(img, "[8] Cross - ENABLE", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
    else if (step == Step::STOP)
        putText(img, "[8] Cross - STOPPING", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
}
void FsmCross::setStep(Step st)
{
    step = st;
    params->setStopReason(control_algorithms::StopReason::CROSS,
                          st == Step::STOP);
}
