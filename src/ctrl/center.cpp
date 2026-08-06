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
 * @file center.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 车辆图像控制中心计算
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ctrl/center.hpp"
#include "ctrl/control_geometry_validator.hpp"
#include "ctrl/perception_geometry_builder.hpp"
#include "ctrl/planned_geometry_builder.hpp"

using namespace cv;
using namespace std;

/**
 * @brief 控制中心计算
 *
 * @param params
 */
void Center::observeLaneWidth(const vector<PointX> &left,
                              const vector<PointX> &right,
                              bool measurementValid)
{
    if (!measurementValid) return;
    std::array<int, ROWSIMAGE> leftByRow;
    std::array<int, ROWSIMAGE> rightByRow;
    leftByRow.fill(-1);
    rightByRow.fill(-1);
    for (const auto &point : left)
        if (point.x >= 0 && point.x < ROWSIMAGE) leftByRow[point.x] = point.y;
    for (const auto &point : right)
        if (point.x >= 0 && point.x < ROWSIMAGE) rightByRow[point.x] = point.y;
    int observedRows = 0;
    const bool profileReady = laneWidthProfileReady();
    for (int row = 0; row < ROWSIMAGE; ++row)
    {
        if (leftByRow[row] < 0 || rightByRow[row] <= leftByRow[row]) continue;
        const float measuredWidth = rightByRow[row] - leftByRow[row];
        if (profileReady && laneWidthProfile[row] > 1.0f &&
            std::abs(measuredWidth - laneWidthProfile[row]) /
                laneWidthProfile[row] > 0.35f)
            continue;
        laneWidthProfile[row] = laneWidthProfile[row] > 1.0f
            ? 0.8f * laneWidthProfile[row] + 0.2f * measuredWidth
            : measuredWidth;
        laneWidthSamples[row] = std::min<uint16_t>(laneWidthSamples[row] + 1, 1000);
        observedRows++;
    }
    if (observedRows >= 30) laneWidthObservationFrames++;
}
bool Center::laneWidthProfileReady() const
{
    const int readyRows = static_cast<int>(std::count_if(
        laneWidthSamples.begin(), laneWidthSamples.end(),
        [](uint16_t samples) { return samples >= 3; }));
    return laneWidthObservationFrames >= 10 && readyRows >= 40;
}

void Center::beginPerceptionRecovery()
{
    laneRecoveryState = control_algorithms::LaneRecoveryState{};
    laneRecoveryState.recovering = true;
    laneRecoveryState.controlValid = false;
    laneRecoveryState.invalidFrames = 0;
    laneRecoveryState.recoveryFrames = 0;
}

void Center::fitting(shared_ptr<Params> &params)
{

    resetControlGeometry(*params);
    const LaneInput laneInput = selectLaneInput(
        params->track->pointsEdgeLeft, params->track->pointsEdgeRight,
        params->pathOverride);
    const ControlGeometrySource selectedSource = selectGeometrySource(
        params->geometryPolicy, laneInput.planned());
    // PLANNED -> PERCEPTION 的控制权切换必须重新连续确认：第一帧感知
    // 即使有效也不放行，连续5帧稳定后才恢复控制，避免规划路径过期后
    // 第一帧立即接管。
    const bool plannedToPerception =
        previousSource == ControlGeometrySource::PLANNED &&
        selectedSource == ControlGeometrySource::PERCEPTION;
    if (plannedToPerception)
        beginPerceptionRecovery();
    const bool plannedPath = selectedSource == ControlGeometrySource::PLANNED;
    const bool visionLaneMode = selectedSource ==
        ControlGeometrySource::PERCEPTION;
    const auto &laneQuality = params->track->quality;
    PlannedLaneWidthModel laneWidthModel;
    laneWidthModel.ready = laneWidthProfileReady();
    if (laneWidthModel.ready)
    {
        float widthSum = 0.0f;
        int widthCount = 0;
        for (int row = 0; row < ROWSIMAGE; ++row)
            if (laneWidthSamples[row] >= 3 && laneWidthProfile[row] > 1.0f)
            {
                laneWidthModel.widthByRow[row] = laneWidthProfile[row];
                widthSum += laneWidthProfile[row];
                ++widthCount;
            }
        if (widthCount > 0)
            laneWidthModel.fallbackWidth = widthSum / widthCount;
    }
    const PerceptionGeometryResult perceptionGeometry =
        buildPerceptionGeometry(*params->track, laneWidthModel, params->config);
    if (visionLaneMode)
    {
        params->ctrl.centerEdge = perceptionGeometry.centerLine;
        recoveryMode = perceptionGeometry.recoveryMode;
        singleSide = perceptionGeometry.singleSide;
        style = "PERCEPTION";
    }
    else if (plannedPath)
    {
        const PlannedGeometryResult planned = buildPlannedGeometry(
            params->pathOverride, params->mode, laneWidthModel);
        params->ctrl.centerEdge = planned.centerLine;
        plannedValidation = planned.validation;
        recoveryMode = planned.valid ? LaneRecoveryMode::STRICT_DUAL
                                     : LaneRecoveryMode::INVALID;
        style = pathSourceName(planned.source);
    }
    usableCenterRows = static_cast<int>(params->ctrl.centerEdge.size());

    bool controlWindowValid = true;
    bool usedWindowControl = false;
    const bool parkingHeadingMode = params->mode == FsmMode::PARK ||
                                    params->ctrl.parking;
    if (visionLaneMode || parkingHeadingMode || plannedPath)
    {
        const bool strictDualHeading = recoveryMode == LaneRecoveryMode::STRICT_DUAL;
        const bool relaxedDualHeading = recoveryMode == LaneRecoveryMode::RELAXED_DUAL;
        const bool degradedHeading = visionLaneMode &&
            recoveryMode != LaneRecoveryMode::INVALID &&
            recoveryMode != LaneRecoveryMode::STRICT_DUAL &&
            laneWidthProfileReady() && params->ctrl.centerEdge.size() >= 20;
        const bool singleEdgeStrict = singleSide == -1
            ? laneQuality.leftReliable : laneQuality.rightReliable;
        const float pathHeadingConfidence = plannedPath
            ? (params->pathOverride.headingConfidence > 0.0f
                ? params->pathOverride.headingConfidence : 0.65f)
            : parkingHeadingMode
            ? params->config.parkingHeadingConfidence
            : (strictDualHeading ? 1.0f
               : (relaxedDualHeading ? 0.65f
                               : (degradedHeading
                                   ? (singleEdgeStrict
                                       ? params->config.singleLaneHeadingConfidence
                                       : params->config.borderClippedHeadingConfidence)
                                   : 0.0f)));
        const int minimumSamples = (parkingHeadingMode || plannedPath) ? 4 : 8;
        const auto centers = control_algorithms::calculateLaneControlCenters(
            params->ctrl.centerEdge, COLSIMAGE / 2.0f, 0.65f, minimumSamples,
            pathHeadingConfidence > 0.0f, params->config.laneHeadingGain,
            params->config.laneHeadingMaxCorrection,
            params->config.laneHeadingFadeError, pathHeadingConfidence);
        nearCenterSamples = centers.nearSamples;
        farCenterSamples = centers.farSamples;
        headingError = centers.headingError;
        headingCorrection = centers.headingCorrection;
        headingConfidence = centers.headingConfidence;
        params->ctrl.laneHeadingCorrection = centers.headingCorrection;
        nearCenterValid = centers.nearValid;
        farCenterValid = centers.farValid;
        if (nearCenterValid)
            nearCenter = static_cast<int>(std::lround(centers.nearCenter));
        if (farCenterValid)
            farCenter = static_cast<int>(std::lround(centers.farCenter));

        controlWindowValid = nearCenterValid;
        if (controlWindowValid)
        {
            usedWindowControl = true;
            params->ctrl.center = std::clamp(
                static_cast<int>(std::lround(centers.controlCenter)),
                0, COLSIMAGE - 1);
        }
    }
    if (!usedWindowControl && !visionLaneMode)
    {
        int controlNum = 1;
        for (const auto &p : params->ctrl.centerEdge)
        {
            const int weight = p.x < ROWSIMAGE / 2 ? ROWSIMAGE / 2 : ROWSIMAGE - p.x;
            controlNum += weight;
            params->ctrl.center += p.y * weight;
        }
        if (controlNum > 1) params->ctrl.center /= controlNum;
    }
    params->ctrl.center = std::clamp(params->ctrl.center, 0, COLSIMAGE - 1);

    // 控制率计算
    if (params->ctrl.centerEdge.size() > 20)
    {
        vector<PointX> centerV;
        int filt = params->ctrl.centerEdge.size() / 5;
        for (int i = filt; i < params->ctrl.centerEdge.size() - filt;
             i++) // 过滤中心点集前后1/5的诱导性
        {
            centerV.push_back(params->ctrl.centerEdge[i]);
        }
        sigmaCenter = sigma(centerV);
    }
    else
        sigmaCenter = 1000;

    // Dynamic continuity (previous-frame dependent) stays in Center; every
    // static validity decision (row alignment, recovery mode, width
    // consistency, near/far samples, base geometry continuity) is owned by
    // PerceptionGeometryBuilder.
    bool degradedCenterContinuous = false;
    const bool degradedMode = recoveryMode == LaneRecoveryMode::RELAXED_DUAL ||
        recoveryMode == LaneRecoveryMode::WEAK_HYBRID ||
        recoveryMode == LaneRecoveryMode::LEFT_SINGLE ||
        recoveryMode == LaneRecoveryMode::RIGHT_SINGLE;
    // 弯道起点宽度模型通常尚未学习完成（laneWidthProfileReady=false），
    // 单边中心线仍由fallbackWidth生成，连续性检查不能因此被跳过。
    if (degradedMode && params->ctrl.centerEdge.size() >= 12)
    {
        const auto limited = control_algorithms::limitSingleLaneCenter(
            params->ctrl.center, lastValidLaneCenter,
            params->config.singleLaneMaxCenterJump,
            params->config.singleLaneCenterStep);
        rawCenterJump = limited.rawJump;
        appliedCenterStep = limited.appliedStep;
        degradedCenterContinuous = limited.valid;
        if (limited.valid) params->ctrl.center = limited.appliedCenter;
    }
    const bool continuityValid = !degradedMode || degradedCenterContinuous;
    const bool candidateValid = plannedPath
        ? controlWindowValid && plannedValidation.valid &&
            params->pathOverride.validFor(laneInput.source) &&
            pathSourceAllowed(laneInput.source, params->mode) &&
            validateControlGeometry(params->ctrl.centerEdge, selectedSource,
                params->geometryPolicy, laneInput.source, params->mode)
        : visionLaneMode
            ? perceptionGeometry.candidateValid && controlWindowValid &&
                continuityValid &&
                validateControlGeometry(params->ctrl.centerEdge, selectedSource,
                    params->geometryPolicy, PathSource::NONE, params->mode)
            : controlWindowValid &&
                validateControlGeometry(params->ctrl.centerEdge, selectedSource,
                    params->geometryPolicy, PathSource::NONE, params->mode);
    const bool perceptionControlled = selectedSource ==
        ControlGeometrySource::PERCEPTION;
    if (perceptionControlled && !params->manualTakeover)
    {
        controlValid = control_algorithms::updateLaneRecovery(
            laneRecoveryState, candidateValid, 5);
        laneInvalidFrames = laneRecoveryState.invalidFrames;
        laneRecoveryFrames = laneRecoveryState.recoveryFrames;
        if (controlValid)
        {
            lastValidLaneCenter = params->ctrl.center;
        }
        else
        {
            params->ctrl.center = lastValidLaneCenter;
        }
    }
    else
    {
        laneRecoveryState = control_algorithms::LaneRecoveryState{};
        controlValid = candidateValid;
        laneInvalidFrames = 0;
        laneRecoveryFrames = 0;
        if (!controlValid && plannedPath)
        {
            rejectedPathSource = laneInput.source;
            params->clearPathOverride(laneInput.source);
            params->ctrl.centerEdge.clear();
        }
    }
    applyControlGeometry(*params, plannedPath, visionLaneMode, laneInput.source);
    previousSource = selectedSource;
    const bool dualStrict = laneQuality.leftReliable && laneQuality.rightReliable;
    const int laneDiagnosticState = dualStrict ? 0 :
        (singleSide != 0 ? singleSide : 2);
    if (visionLaneMode && laneDiagnosticState != previousLaneDiagnosticState)
    {
        const char *sideName = laneDiagnosticState < 0 ? "left" :
            (laneDiagnosticState == 1 ? "right" :
             (laneDiagnosticState == 0 ? "dual" : "invalid"));
        std::cout << "[Lane] singleSide=" << sideName
                  << " leftStrict=" << laneQuality.leftReliable
                  << " rightStrict=" << laneQuality.rightReliable
                  << " leftSingleUsable=" << laneQuality.leftSingleUsable
                  << " rightSingleUsable=" << laneQuality.rightSingleUsable
                  << " leftInteriorPoints=" << laneQuality.leftInteriorPoints
                  << " rightInteriorPoints=" << laneQuality.rightInteriorPoints
                  << " leftBorderRatio=" << laneQuality.leftBorderRatio
                  << " rightBorderRatio=" << laneQuality.rightBorderRatio
                  << " laneWidthReady=" << laneWidthProfileReady()
                  << " rawCenterJump=" << rawCenterJump
                  << " appliedCenterStep=" << appliedCenterStep
                  << " controlValid=" << controlValid << std::endl;
        previousLaneDiagnosticState = laneDiagnosticState;
    }
}

/**
 * @brief 显示赛道线识别结果
 *
 * @param img 需要叠加显示的图像
 */
void Center::drawImage(shared_ptr<Params> &params, Mat &img)
{
    // 赛道边缘绘制
    for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        circle(img, Point(params->track->pointsEdgeLeft[i].y, params->track->pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点

    for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        circle(img, Point(params->track->pointsEdgeRight[i].y, params->track->pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点

    // 绘制中心点集
    for (int i = 0; i < params->ctrl.centerEdge.size(); i++)
        circle(img, Point(params->ctrl.centerEdge[i].y, params->ctrl.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);

    // 绘制加权控制中心：方向
    Rect rect(params->ctrl.center, ROWSIMAGE - 20, 10, 20);
    rectangle(img, rect, Scalar(0, 0, 255), cv::FILLED);

    // 详细控制参数显示
    int dis = 20;
    string str;

    str = "Edge: " + doble2String(params->track->stdevLeft, 1) + " | " +
          doble2String(params->track->stdevRight, 1);
    putText(img, str, Point(COLSIMAGE - 100, 2 * dis), FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 0, 255), 1); // 斜率：左|右

    putText(img, to_string(params->ctrl.center), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40),
            FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1); // 中心

    // 调试画面：车道模式 + 中心线点数 + 近场点数（便于直接判断失败原因）
    putText(img, string("lane=") + laneRecoveryModeName(recoveryMode) +
            " samples=" + to_string(usableCenterRows) +
            " near=" + to_string(nearCenterSamples),
            Point(10, 3 * dis), FONT_HERSHEY_PLAIN, 0.7,
            Scalar(0, 0, 255), 1);

    // 绘制FSM状态
    showMode(img, params->mode);
}

/**
 * @brief 显示FSM状态
 *
 * @param img
 * @param mode
 */
void Center::showMode(Mat &img, FsmMode mode)
{
    std::string str = "", logo = "";
    switch (mode)
    {
    case FsmMode::FORK:
        str = "[Fork]";
        logo = "F";
        break;
    case FsmMode::PARK:
        str = "[Park]";
        logo = "P";
        break;
    case FsmMode::BUSY:
        str = "[Busy]";
        logo = "B";
        break;
    case FsmMode::SLOW:
        str = "[Slow]";
        logo = "S";
        break;
    case FsmMode::STOP:
        str = "[Stop]";
        logo = "X";
        break;
    case FsmMode::CROSS:
        str = "[Cross]";
        logo = "X";
        break;
    }

    if (mode != FsmMode::NORMAL)
    {
        putText(img, str, Point(COLSIMAGE / 2 - 50, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
        putText(img, logo, Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
    }
    else
    {
        putText(img, "[Normal]", Point(COLSIMAGE / 2 - 50, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
    }
}

