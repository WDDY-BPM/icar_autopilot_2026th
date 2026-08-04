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
    for (int row = 0; row < ROWSIMAGE; ++row)
    {
        if (leftByRow[row] < 0 || rightByRow[row] <= leftByRow[row]) continue;
        const float measuredWidth = rightByRow[row] - leftByRow[row];
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

void Center::fitting(shared_ptr<Params> &params)
{

    sigmaCenter = 0;
    params->ctrl.center = COLSIMAGE / 2; // 控制中心
    nearCenter = COLSIMAGE / 2;
    farCenter = COLSIMAGE / 2;
    headingError = 0.0f;
    headingCorrection = 0.0f;
    headingConfidence = 0.0f;
    params->ctrl.laneHeadingCorrection = 0.0f;
    nearCenterSamples = 0;
    farCenterSamples = 0;
    nearCenterValid = false;
    farCenterValid = false;
    const bool visionLaneMode = !params->ctrl.fitting &&
        (params->mode == FsmMode::NORMAL || params->mode == FsmMode::CURVE ||
         params->mode == FsmMode::CROSS || params->mode == FsmMode::STOP ||
         params->mode == FsmMode::SLOW || params->mode == FsmMode::STATION);
    if (!params->ctrl.fitting)           // 除停车场特殊绘制
    {
        if (visionLaneMode && !params->track->quality.leftReliable)
            params->track->pointsEdgeLeft.clear();
        if (visionLaneMode && !params->track->quality.rightReliable)
            params->track->pointsEdgeRight.clear();
        params->ctrl.centerEdge.clear();
        vector<PointX> v_center(4); // 三阶贝塞尔曲线
        style = "STRIGHT";

        // 边缘斜率重计算（边缘修正之后）
        params->track->stdevLeft = params->track->stdevEdgeCal(params->track->pointsEdgeLeft, ROWSIMAGE);
        params->track->stdevRight = params->track->stdevEdgeCal(params->track->pointsEdgeRight, ROWSIMAGE);

        // 边缘有效行优化
        if ((params->track->stdevLeft < 80 && params->track->stdevRight > 30) || (params->track->stdevLeft > 60 && params->track->stdevRight < 50))
        {
            validRowsCal(params->track->pointsEdgeLeft, params->track->pointsEdgeRight); // 边缘有效行计算
            params->track->pointsEdgeLeft.resize(validRowsLeft);
            params->track->pointsEdgeRight.resize(validRowsRight);
        }

        if (params->track->pointsEdgeLeft.size() > 4 &&
            params->track->pointsEdgeRight.size() > 4)
        {
            params->ctrl.centerEdge = buildRowAlignedCenter(
                params->track->pointsEdgeLeft,
                params->track->pointsEdgeRight,
                false);
            style = "STRIGHT";
        }
        // Left single edge
        else if ((params->track->pointsEdgeLeft.size() > 0 && params->track->pointsEdgeRight.size() <= 4) ||
                 (params->track->pointsEdgeLeft.size() > 0 && params->track->pointsEdgeRight.size() > 0 &&
                  params->track->pointsEdgeLeft[0].x - params->track->pointsEdgeRight[0].x > ROWSIMAGE / 2))
        {
            style = "RIGHT";
            params->ctrl.centerEdge = centerCompute(params->track->pointsEdgeLeft, 0);
        }
        // 右单边
        else if ((params->track->pointsEdgeRight.size() > 0 && params->track->pointsEdgeLeft.size() <= 4) ||
                 (params->track->pointsEdgeRight.size() > 0 && params->track->pointsEdgeLeft.size() > 0 &&
                  params->track->pointsEdgeRight[0].x - params->track->pointsEdgeLeft[0].x > ROWSIMAGE / 2))
        {
            style = "LEFT";
            params->ctrl.centerEdge = centerCompute(params->track->pointsEdgeRight, 1);
        }
        else if (params->track->pointsEdgeLeft.size() > 4 && params->track->pointsEdgeRight.size() == 0) // 左单边
        {
            v_center[0] = {params->track->pointsEdgeLeft[0].x, (params->track->pointsEdgeLeft[0].y + COLSIMAGE - 1) / 2};

            v_center[1] = {params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() / 3].x,
                           (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() / 3].y + COLSIMAGE - 1) / 2};

            v_center[2] = {
                params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 2 / 3].x,
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 2 / 3].y + COLSIMAGE - 1) / 2};

            v_center[3] = {params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x,
                           (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].y + COLSIMAGE - 1) / 2};

            params->ctrl.centerEdge = Bezier(0.02, v_center);
            style = "RIGHT";
        }
        else if (params->track->pointsEdgeLeft.size() == 0 &&
                 params->track->pointsEdgeRight.size() > 4) // 右单边
        {
            v_center[0] = {params->track->pointsEdgeRight[0].x, params->track->pointsEdgeRight[0].y / 2};

            v_center[1] = {params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() / 3].x,
                           params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() / 3].y / 2};

            v_center[2] = {
                params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 2 / 3].x,
                params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 2 / 3].y / 2};

            v_center[3] = {params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x,
                           params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].y / 2};

            params->ctrl.centerEdge = Bezier(0.02, v_center);
            style = "LEFT";
        }
    }

    bool controlWindowValid = true;
    bool usedWindowControl = false;
    const bool parkingHeadingMode = params->mode == FsmMode::PARK ||
                                    params->ctrl.parking;
    if (visionLaneMode || parkingHeadingMode)
    {
        const bool dualLaneHeading =
            params->track->quality.leftReliable &&
            params->track->quality.rightReliable &&
            params->track->quality.commonRows >= 20;
        const bool singleLaneHeading = visionLaneMode &&
            (params->track->quality.leftReliable !=
             params->track->quality.rightReliable) &&
            laneWidthProfileReady() && params->ctrl.centerEdge.size() >= 20;
        const float pathHeadingConfidence = parkingHeadingMode
            ? params->config.parkingHeadingConfidence
            : (dualLaneHeading ? 1.0f
                               : (singleLaneHeading
                                   ? params->config.singleLaneHeadingConfidence
                                   : 0.0f));
        const int minimumSamples = parkingHeadingMode ? 4 : 8;
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
    if (params->ctrl.center > COLSIMAGE)
        params->ctrl.center = COLSIMAGE;
    else if (params->ctrl.center < 0)
        params->ctrl.center = 0;

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

    const bool strictLaneMode = params->mode == FsmMode::NORMAL ||
                                params->mode == FsmMode::CURVE ||
                                params->mode == FsmMode::CROSS ||
                                params->mode == FsmMode::STOP ||
                                params->mode == FsmMode::SLOW ||
                                params->mode == FsmMode::STATION;
    // Two independently reliable, bottom-covering edges provide a usable
    // centerline even on a tight curve. Aggregate quality.valid also contains
    // straight-road temporal/width thresholds and must not reject that path.
    const bool bothValid = params->track->quality.leftReliable &&
                           params->track->quality.rightReliable &&
                           params->track->quality.coversBottom &&
                           params->track->quality.commonRows >= 20;
    const bool singleCenterContinuous =
        control_algorithms::isSingleLaneCenterContinuous(
            params->ctrl.center, lastValidLaneCenter, 15);
    const bool singleValid =
        (params->track->quality.leftReliable != params->track->quality.rightReliable) &&
        laneWidthProfileReady() && params->ctrl.centerEdge.size() >= 20 &&
        singleCenterContinuous;
    const bool candidateValid = controlWindowValid &&
                                params->ctrl.centerEdge.size() >= 20 &&
                                (bothValid || singleValid);
    if (strictLaneMode && !params->ctrl.parking && !params->manualTakeover)
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
            if (laneInvalidFrames > 6)
                params->ctrl.center = COLSIMAGE / 2;
        }
    }
    else
    {
        // FSM-generated parking/fork/construction paths use their own validity.
        laneRecoveryState = control_algorithms::LaneRecoveryState{};
        controlValid = !params->ctrl.centerEdge.empty();
        laneInvalidFrames = 0;
        laneRecoveryFrames = 0;
        if (controlValid)
            lastValidCenter = params->ctrl.center;
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
    case FsmMode::CURVE:
        str = "[Curve]";
        logo = "C";
        break;
    case FsmMode::FINE:
        str = "[Fine]";
        logo = "X";
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
    vector<PointX> center;
    center.reserve(pointsEdge.size() / 2);
    for (size_t i = 0; i < pointsEdge.size(); i += 2)
    {
        const int row = pointsEdge[i].x;
        if (row < 0 || row >= ROWSIMAGE || laneWidthProfile[row] <= 1.0f)
            continue;
        const float halfWidth = laneWidthProfile[row] * 0.5f;
        const int column = side == 0
            ? static_cast<int>(std::lround(pointsEdge[i].y + halfWidth))
            : static_cast<int>(std::lround(pointsEdge[i].y - halfWidth));
        if (column > 0 && column < COLSIMAGE)
            center.emplace_back(row, column);
    }
    return center;
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
