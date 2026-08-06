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
 * @file center.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 车辆图像控制中心计算
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include <array>
#include <iterator>
#include "utils/tools.hpp"
#include "utils/params.hpp"
#include "ctrl/control_algorithms.hpp"
#include "runtime/planned_path_validation.hpp"
#include "ctrl/control_geometry.hpp"
#include "ctrl/perception_geometry_builder.hpp"

/**
 * @brief 控制中心处理类
 *
 */
class Center
{

public:
    double sigmaCenter = 0;
    bool controlValid = false;
    PathSource rejectedPathSource = PathSource::NONE;
    ControlGeometry geometry;
    PlannedPathValidation plannedValidation;
    int laneInvalidFrames = 0;
    int laneRecoveryFrames = 0;
    int nearCenter = COLSIMAGE / 2;
    int farCenter = COLSIMAGE / 2;
    float headingError = 0.0f;
    float headingCorrection = 0.0f;
    float headingConfidence = 0.0f;
    int nearCenterSamples = 0;
    int farCenterSamples = 0;
    bool nearCenterValid = false;
    bool farCenterValid = false;
    int singleSide = 0; // -1 left edge, +1 right edge, 0 dual/invalid
    int rawCenterJump = 0;
    int appliedCenterStep = 0;
    int usableCenterRows = 0;
    LaneRecoveryMode recoveryMode = LaneRecoveryMode::INVALID;

    void fitting(shared_ptr<Params> &params);
    void observeLaneWidth(const vector<PointX> &left, const vector<PointX> &right,
                          bool measurementValid);
    bool laneWidthProfileReady() const;
    void drawImage(shared_ptr<Params> &params, Mat &img);

private:
    string style = "";

    void resetControlGeometry(Params &params);
    void applyControlGeometry(Params &params, bool plannedPath,
                              bool perceptionPath, PathSource pathSource);

    void showMode(Mat &img, FsmMode mode);
    std::array<float, ROWSIMAGE> laneWidthProfile{};
    std::array<uint16_t, ROWSIMAGE> laneWidthSamples{};
    int laneWidthObservationFrames = 0;
    int lastValidLaneCenter = COLSIMAGE / 2;
    int previousLaneDiagnosticState = -99;
    control_algorithms::LaneRecoveryState laneRecoveryState;
};
