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

vector<PointX> Center::buildDegradedLaneCenter(const vector<PointX> &left,
                                               const vector<PointX> &right)
{
    return control_algorithms::buildDegradedLaneCenter(
        left, right, laneWidthProfile, ROWSIMAGE, COLSIMAGE);
}

bool Center::laneWidthConsistent(const vector<PointX> &left,
                                 const vector<PointX> &right) const
{
    if (!laneWidthProfileReady()) return true;
    std::array<int, ROWSIMAGE> leftByRow, rightByRow;
    leftByRow.fill(-1); rightByRow.fill(-1);
    for (const auto &point : left)
        if (point.x >= 0 && point.x < ROWSIMAGE) leftByRow[point.x] = point.y;
    for (const auto &point : right)
        if (point.x >= 0 && point.x < ROWSIMAGE) rightByRow[point.x] = point.y;
    vector<float> errors;
    for (int row = 0; row < ROWSIMAGE; ++row)
        if (leftByRow[row] >= 0 && rightByRow[row] > leftByRow[row] &&
            laneWidthProfile[row] > 1.0f)
            errors.push_back(std::abs((rightByRow[row] - leftByRow[row]) -
                laneWidthProfile[row]) / laneWidthProfile[row]);
    if (errors.size() < 12) return false;
    std::nth_element(errors.begin(), errors.begin() + errors.size() / 2, errors.end());
    return errors[errors.size() / 2] <= 0.35f;
}

void Center::fitting(shared_ptr<Params> &params)
{

    resetControlGeometry(*params);
    const LaneInput laneInput = selectLaneInput(
        params->track->pointsEdgeLeft, params->track->pointsEdgeRight,
        params->pathOverride);
    const ControlGeometrySource selectedSource = selectGeometrySource(
        params->geometryPolicy, laneInput.planned());
    const bool plannedPath = selectedSource == ControlGeometrySource::PLANNED;
    const bool visionLaneMode = selectedSource ==
        ControlGeometrySource::PERCEPTION;
    const auto &laneQuality = params->track->quality;
    const vector<PointX> detectedLeft = params->track->pointsEdgeLeft;
    const vector<PointX> detectedRight = params->track->pointsEdgeRight;
    LaneWidthModel laneWidthModel;
    laneWidthModel.ready = laneWidthProfileReady();
    if (laneWidthModel.ready)
    {
        float widthSum = 0.0f;
        int widthCount = 0;
        for (int row = 0; row < ROWSIMAGE; ++row)
            if (laneWidthSamples[row] >= 3 && laneWidthProfile[row] > 1.0f)
            {
                widthSum += laneWidthProfile[row];
                ++widthCount;
            }
        if (widthCount > 0)
            laneWidthModel.nominalWidth = widthSum / widthCount;
    }
    const PerceptionGeometryResult perceptionGeometry =
        buildPerceptionGeometry(*params->track, laneWidthModel, params->config);
    if (visionLaneMode)
    {
        if (laneQuality.leftReliable && laneQuality.rightReliable)
        {
            singleSide = 0;
            selectedRecoverySide = 0;
            const bool relaxedGeometry = laneQuality.commonRows >= 20 &&
                laneQuality.coversBottom && laneQuality.edgeJump <= 30.0f &&
                laneQuality.widthVariation <= 0.30f && laneQuality.centerJump <= 45.0f &&
                laneWidthConsistent(detectedLeft, detectedRight);
            recoveryMode = laneQuality.valid ? LaneRecoveryMode::STRICT_DUAL :
                (relaxedGeometry ? LaneRecoveryMode::RELAXED_DUAL : LaneRecoveryMode::INVALID);
        }
        else if (laneQuality.leftReliable)
        {
            singleSide = -1;
            selectedRecoverySide = -1;
            recoveryMode = LaneRecoveryMode::LEFT_SINGLE;
        }
        else if (laneQuality.rightReliable)
        {
            singleSide = 1;
            selectedRecoverySide = 1;
            recoveryMode = LaneRecoveryMode::RIGHT_SINGLE;
        }
        else if (laneQuality.leftSingleUsable && laneQuality.rightSingleUsable)
            recoveryMode = LaneRecoveryMode::WEAK_HYBRID;
        else if (laneQuality.leftSingleUsable != laneQuality.rightSingleUsable)
        {
            singleSide = laneQuality.leftSingleUsable ? -1 : 1;
            selectedRecoverySide = singleSide;
            recoveryMode = singleSide < 0 ? LaneRecoveryMode::LEFT_SINGLE :
                                            LaneRecoveryMode::RIGHT_SINGLE;
        }
        if (recoveryMode == LaneRecoveryMode::INVALID)
            selectedRecoverySide = 0;
    }
    if (visionLaneMode)
    {
        params->ctrl.centerEdge.clear();
        const auto interiorOnly = [](const vector<PointX> &edge) {
            vector<PointX> interior;
            std::copy_if(edge.begin(), edge.end(), std::back_inserter(interior),
                [](const PointX &point) {
                    return point.y > 2 && point.y < COLSIMAGE - 3;
                });
            return interior;
        };
        const auto nearSampleCount = [](const vector<PointX> &centerline) {
            return static_cast<int>(std::count_if(centerline.begin(), centerline.end(),
                [](const PointX &point) { return point.x >= 176 && point.x <= 220; }));
        };
        const auto singleCandidateScore = [](const vector<PointX> &centerline) {
            const auto windows = control_algorithms::calculateLaneControlCenters(
                centerline, COLSIMAGE / 2.0f, 0.65f, 8, false);
            const bool continuous = std::adjacent_find(
                centerline.begin(), centerline.end(),
                [](const PointX &a, const PointX &b) {
                    return std::abs(a.y - b.y) > 45;
                }) == centerline.end();
            if (!windows.nearValid || !continuous)
                return -1;
            return windows.nearSamples * 100 + windows.farSamples * 10 +
                std::min(static_cast<int>(centerline.size()), 99);
        };
        if (recoveryMode == LaneRecoveryMode::STRICT_DUAL ||
            recoveryMode == LaneRecoveryMode::RELAXED_DUAL)
            params->ctrl.centerEdge = buildRowAlignedCenter(
                detectedLeft, detectedRight, false);
        else if (recoveryMode == LaneRecoveryMode::LEFT_SINGLE)
            params->ctrl.centerEdge = centerCompute(interiorOnly(detectedLeft), 0);
        else if (recoveryMode == LaneRecoveryMode::RIGHT_SINGLE)
            params->ctrl.centerEdge = centerCompute(interiorOnly(detectedRight), 1);
        else if (recoveryMode == LaneRecoveryMode::WEAK_HYBRID)
        {
            params->ctrl.centerEdge = buildDegradedLaneCenter(detectedLeft, detectedRight);
            if (params->ctrl.centerEdge.size() < 12 ||
                nearSampleCount(params->ctrl.centerEdge) < 8)
            {
                const vector<PointX> leftCandidate = centerCompute(interiorOnly(detectedLeft), 0);
                const vector<PointX> rightCandidate = centerCompute(interiorOnly(detectedRight), 1);
                const int leftScore = singleCandidateScore(leftCandidate);
                const int rightScore = singleCandidateScore(rightCandidate);
                const bool heldCandidateValid = selectedRecoverySide < 0
                    ? leftScore >= 0 : selectedRecoverySide > 0 && rightScore >= 0;
                if (recoverySideHoldFrames <= 0 || selectedRecoverySide == 0 ||
                    !heldCandidateValid)
                {
                    if (leftScore != rightScore)
                        selectedRecoverySide = leftScore > rightScore ? -1 : 1;
                    else if (laneQuality.leftInteriorPoints != laneQuality.rightInteriorPoints)
                        selectedRecoverySide = laneQuality.leftInteriorPoints >
                            laneQuality.rightInteriorPoints ? -1 : 1;
                    else
                        selectedRecoverySide = laneQuality.leftBorderRatio <=
                            laneQuality.rightBorderRatio ? -1 : 1;
                    recoverySideHoldFrames = 3;
                }
                else
                    recoverySideHoldFrames--;
                params->ctrl.centerEdge = selectedRecoverySide < 0
                    ? leftCandidate : rightCandidate;
                singleSide = selectedRecoverySide;
            }
            else
            {
                selectedRecoverySide = 0;
                recoverySideHoldFrames = 0;
            }
        }
    }
    else if (plannedPath)
    {
        const PlannedGeometryResult planned = buildPlannedGeometry(
            params->pathOverride, params->mode);
        params->ctrl.centerEdge = planned.centerLine;
        plannedValidation = planned.validation;
        recoveryMode = planned.valid ? LaneRecoveryMode::STRICT_DUAL
                                     : LaneRecoveryMode::INVALID;
        style = pathSourceName(planned.source);
    }
    if (visionLaneMode && params->ctrl.centerEdge.empty() &&
        perceptionGeometry.candidateValid)
    {
        params->ctrl.centerEdge = perceptionGeometry.centerLine;
        singleSide = perceptionGeometry.singleSide;
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

    const bool strictLaneMode = visionLaneMode;
    // Two independently reliable, bottom-covering edges provide a usable
    // centerline even on a tight curve. Aggregate quality.valid also contains
    // straight-road temporal/width thresholds and must not reject that path.
    bool degradedCenterContinuous = false;
    const bool degradedMode = recoveryMode == LaneRecoveryMode::RELAXED_DUAL ||
        recoveryMode == LaneRecoveryMode::WEAK_HYBRID ||
        recoveryMode == LaneRecoveryMode::LEFT_SINGLE ||
        recoveryMode == LaneRecoveryMode::RIGHT_SINGLE;
    if (degradedMode && laneWidthProfileReady() &&
        params->ctrl.centerEdge.size() >= 12)
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
    const bool strictValid = recoveryMode == LaneRecoveryMode::STRICT_DUAL &&
        laneQuality.valid && params->ctrl.centerEdge.size() >= 20;
    const bool relaxedValid = recoveryMode == LaneRecoveryMode::RELAXED_DUAL &&
        params->ctrl.centerEdge.size() >= 20 && degradedCenterContinuous;
    const bool weakHybridValid = recoveryMode == LaneRecoveryMode::WEAK_HYBRID &&
        params->ctrl.centerEdge.size() >= 12 && nearCenterSamples >= 8 &&
        degradedCenterContinuous;
    const bool singleValid = (recoveryMode == LaneRecoveryMode::LEFT_SINGLE ||
        recoveryMode == LaneRecoveryMode::RIGHT_SINGLE) &&
        params->ctrl.centerEdge.size() >= 20 && degradedCenterContinuous;
    const bool candidateValid = plannedPath
        ? controlWindowValid && plannedValidation.valid &&
            params->pathOverride.validFor(laneInput.source) &&
            pathSourceAllowed(laneInput.source, params->mode) &&
            validateControlGeometry(params->ctrl.centerEdge, selectedSource,
                params->geometryPolicy, laneInput.source, params->mode)
        : controlWindowValid &&
            (strictValid || relaxedValid || weakHybridValid || singleValid) &&
            validateControlGeometry(params->ctrl.centerEdge, selectedSource,
                params->geometryPolicy, PathSource::NONE, params->mode);
    if (strictLaneMode && !plannedPath && !params->ctrl.parking &&
        !params->manualTakeover)
    {
        controlValid = control_algorithms::updateLaneRecovery(
            laneRecoveryState, candidateValid, 5);
        laneInvalidFrames = laneRecoveryState.invalidFrames;
        laneRecoveryFrames = laneRecoveryState.recoveryFrames;
        if (controlValid)
        {
            lastValidCenter = params->ctrl.center;
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
        if (controlValid)
            lastValidCenter = params->ctrl.center;
        else if (plannedPath)
        {
            rejectedPathSource = laneInput.source;
            params->clearPathOverride(laneInput.source);
            params->ctrl.centerEdge.clear();
        }
    }
    applyControlGeometry(*params, plannedPath, visionLaneMode, laneInput.source);
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

/**
 * @brief 搜索十字赛道突变行（左下）
 *
 * @param pointsEdgeLeft
 * @return uint16_t
 */
uint16_t Center::searchBreakLeftDown(vector<PointX> pointsEdgeLeft)
{
    uint16_t counter = 0;

    for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
    {
        if (pointsEdgeLeft[i].y >= 2)
        {
            counter++;
            if (counter > 3)
            {
                return i - 2;
            }
        }
        else
            counter = 0;
    }

    return 0;
}

/**
 * @brief 搜索十字赛道突变行（右下）
 *
 * @param pointsEdgeRight
 * @return uint16_t
 */
uint16_t Center::searchBreakRightDown(vector<PointX> pointsEdgeRight)
{
    uint16_t counter = 0;

    for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
    {
        if (pointsEdgeRight[i].y < COLSIMAGE - 2)
        {
            counter++;
            if (counter > 3)
            {
                return i - 2;
            }
        }
        else
            counter = 0;
    }

    return 0;
}

/**
 * @brief 赛道中心点计算：单边控制
 *
 * @param pointsEdge 赛道边缘点集
 * @param side 单边类型：左边0/右边1
 * @return vector<PointX>
 */
vector<PointX> Center::buildRowAlignedCenter(const vector<PointX> &left,
                                             const vector<PointX> &right,
                                             bool updateHistory)
{
    std::array<int, ROWSIMAGE> leftByRow;
    std::array<int, ROWSIMAGE> rightByRow;
    leftByRow.fill(-1);
    rightByRow.fill(-1);
    for (const auto &point : left)
        if (point.x >= 0 && point.x < ROWSIMAGE)
            leftByRow[point.x] = point.y;
    for (const auto &point : right)
        if (point.x >= 0 && point.x < ROWSIMAGE)
            rightByRow[point.x] = point.y;

    vector<PointX> center;
    center.reserve(ROWSIMAGE);
    for (int row = ROWSIMAGE - 1; row >= 0; --row)
    {
        if (leftByRow[row] < 0 || rightByRow[row] <= leftByRow[row])
            continue;
        const float measuredWidth = rightByRow[row] - leftByRow[row];
        if (updateHistory)
            laneWidthProfile[row] = laneWidthProfile[row] > 1.0f
                ? 0.8f * laneWidthProfile[row] + 0.2f * measuredWidth
                : measuredWidth;
        center.emplace_back(row,
            static_cast<int>(std::lround((leftByRow[row] + rightByRow[row]) * 0.5f)));
    }

    // Remove isolated horizontal spikes without changing row alignment.
    if (center.size() >= 5)
    {
        vector<PointX> filtered = center;
        for (size_t i = 2; i + 2 < center.size(); ++i)
        {
            std::array<int, 5> columns = {
                center[i - 2].y, center[i - 1].y, center[i].y,
                center[i + 1].y, center[i + 2].y};
            std::sort(columns.begin(), columns.end());
            filtered[i].y = columns[2];
        }
        center.swap(filtered);
    }
    return center;
}

vector<PointX> Center::centerCompute(vector<PointX> pointsEdge, int side)
{
    return control_algorithms::reconstructSingleLaneCenter(
        pointsEdge, laneWidthProfile, side == 0, ROWSIMAGE, COLSIMAGE);
}
/**
 * @brief 边缘有效行计算：左/右
 *
 * @param pointsEdgeLeft
 * @param pointsEdgeRight
 */
void Center::validRowsCal(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight)
{
    int counter = 0;
    if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
    {
        uint16_t rowBreakLeft = searchBreakLeftDown(pointsEdgeLeft);                                           // 右边缘上升拐点
        uint16_t rowBreakRight = searchBreakRightDown(pointsEdgeRight);                                        // 右边缘上升拐点
        if (pointsEdgeRight[pointsEdgeRight.size() - 1].y < COLSIMAGE / 2 && rowBreakRight - rowBreakLeft > 5) // 左弯道
        {
            if (pointsEdgeLeft.size() > rowBreakRight) // 左边缘有效行重新搜索
            {
                for (int i = rowBreakRight; i < pointsEdgeLeft.size(); i++)
                {
                    if (pointsEdgeLeft[i].y < 1)
                    {
                        counter++;
                        if (counter >= 3)
                        {
                            pointsEdgeLeft.resize(i - 3);
                        }
                    }
                    else
                        counter = 0;
                }
            }
        }

        else if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y > COLSIMAGE / 2 && rowBreakLeft - rowBreakRight > 5) // 右弯道
        {

            if (pointsEdgeRight.size() > rowBreakLeft) // 右边缘有效行重新搜索
            {
                for (int i = rowBreakLeft; i < pointsEdgeRight.size(); i++)
                {
                    if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                    {
                        counter++;
                        if (counter >= 3)
                        {
                            pointsEdgeRight.resize(i - 3);
                        }
                    }
                    else
                        counter = 0;
                }
            }
        }
    }

    // 左边有效行
    validRowsLeft = 0;
    int count = 0;
    if (pointsEdgeLeft.size() > 10)
    {
        for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeLeft[i].y > 2)
            {
                count++;
                if (count > 4)
                {
                    validRowsLeft = i + 4;
                    break;
                }
            }
        }
    }

    // 右边有效行
    validRowsRight = 0;
    if (pointsEdgeRight.size() > 1)
    {
        for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeRight[i].y <= COLSIMAGE - 2 &&
                pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
            if (pointsEdgeRight[i].y >= COLSIMAGE - 2 &&
                pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
        }
    }
}
