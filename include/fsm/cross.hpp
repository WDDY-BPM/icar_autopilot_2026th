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
 * @file cross.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 斑马线停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 斑马线停车控制
 *
 */
class FsmCross : public FSMState
{
public:
  FsmCross(std::shared_ptr<Params> par);
  ~FsmCross();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();
private:
  /**
   * @brief 场景状态
   *
   */
  enum Step
  {
    NONE = 0, // AI未识别
    ENABLE,   // 场景使能
    STOP      // 停车
  };

  Step step = Step::NONE;
  int countInit = 0;
  control_algorithms::CrossConfirmationState confirmation;
  void setStep(Step st);
};
