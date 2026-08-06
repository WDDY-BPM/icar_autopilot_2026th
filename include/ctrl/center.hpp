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

#define DIS_MOVE 48       // 偏移距离，对应赛道距离的一般，在我的打表软件中默认48像素对应20cm
#define MAX_POINT_NUM 240 // 无需修改
#define DIS_SECTION 75    // 使用centerMove时每一段点集的最大长度

enum class LaneRecoveryMode
{
    INVALID, STRICT_DUAL, RELAXED_DUAL, WEAK_HYBRID,
    LEFT_SINGLE, RIGHT_SINGLE
};

/**
 * @brief 控制中心处理类
 *
 */
class Center
{

public:
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
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
    int selectedRecoverySide = 0;

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
    uint16_t searchBreakLeftDown(vector<PointX> pointsEdgeLeft);
    uint16_t searchBreakRightDown(vector<PointX> pointsEdgeRight);
    vector<PointX> centerCompute(vector<PointX> pointsEdge, int side);
    vector<PointX> buildDegradedLaneCenter(const vector<PointX> &left,
                                           const vector<PointX> &right);
    bool laneWidthConsistent(const vector<PointX> &left,
                             const vector<PointX> &right) const;
    void validRowsCal(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight);
    vector<PointX> buildRowAlignedCenter(const vector<PointX> &left,
                                         const vector<PointX> &right,
                                         bool updateHistory);
    std::array<float, ROWSIMAGE> laneWidthProfile{};
    std::array<uint16_t, ROWSIMAGE> laneWidthSamples{};
    int laneWidthObservationFrames = 0;
    int lastValidCenter = COLSIMAGE / 2;
    int lastValidLaneCenter = COLSIMAGE / 2;
    int previousLaneDiagnosticState = -99;
    int recoverySideHoldFrames = 0;
    control_algorithms::LaneRecoveryState laneRecoveryState;
};
