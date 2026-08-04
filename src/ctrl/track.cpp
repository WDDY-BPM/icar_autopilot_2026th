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
 * @file track.hpp
 * @author Leo (liaotengjun@saishukeji.com)
 * @brief 车道线检测
 * @version 0.1
 * @date 2025-07-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ctrl/track.hpp"
#include <limits>

using namespace cv;
using namespace std;

/**
 * @brief 赛道线识别
 *
 * @param imageBinary 赛道识别基准图像
 */
void Track::handle(Mat img)
{
    imgShare = img;
    handle(false, 0);
}

/**
 * @brief 赛道线识别
 *
 * @param isResearch 是否重复搜索
 * @param rowStart 边缘搜索起始行
 */
void Track::handle(bool isResearch, uint16_t rowStart)
{
    // A failed search must never leave the previous frame marked valid.
    if (!isResearch)
        quality = LaneQuality{};
    bool flagStartBlock = true;                    // 搜索到色块起始行的标志（行）
    int counterSearchRows = pointsEdgeLeft.size(); // 搜索行计数
    int startBlock[30];                            // 色块起点（行）
    int endBlock[30];                              // 色块终点（行）
    int counterBlock = 0;                          // 色块计数器（行）
    PointX pointSpurroad;                          // 岔路坐标
    int counterSpurroad = 0;                       // 岔路识别标志
    bool spurroadEnable = false;
    int initialLeft = -1;
    int initialRight = -1;
    int initialStableRows = 0;
    int previousInternalGap = -1;
    int expandingGapRows = 0;

    if (rowCutUp > ROWSIMAGE / 4)
        rowCutUp = ROWSIMAGE / 4;
    if (rowCutBottom > ROWSIMAGE / 4)
        rowCutBottom = ROWSIMAGE / 4;

    if (!isResearch)
    {
        pointsEdgeLeft.clear();              // 初始化边缘结果
        pointsEdgeRight.clear();             // 初始化边缘结果
        widthBlock.clear();                  // 初始化色块数据
        spurroad.clear();                    // 岔路信息
        validRowsLeft = 0;                   // 边缘有效行数（左）
        validRowsRight = 0;                  // 边缘有效行数（右）
        flagStartBlock = true;               // 搜索到色块起始行的标志（行）
        rowStart = ROWSIMAGE - rowCutBottom; // 默认底部起始行
    }
    else
    {
        if (pointsEdgeLeft.size() > rowStart)
            pointsEdgeLeft.resize(rowStart);
        if (pointsEdgeRight.size() > rowStart)
            pointsEdgeRight.resize(rowStart);
        if (widthBlock.size() > rowStart)
        {
            widthBlock.resize(rowStart);
            if (rowStart > 1)
                rowStart = widthBlock[rowStart - 1].x - 2;
        }

        flagStartBlock = false; // 搜索到色块起始行的标志（行）
    }

    //  开始识别赛道左右边缘
    for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
    {
        counterBlock = 0; // 色块计数器清空
                          // 搜索色（block）块信息
        if (imgShare.at<uchar>(row, 0) > 127)
        {
            startBlock[counterBlock] = 0;
        }
        for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
        {
            if (imgShare.at<uchar>(row, col) > 127 &&
                imgShare.at<uchar>(row, col - 1) <= 127)
            {
                startBlock[counterBlock] = col;
            }
            else
            {
                if (imgShare.at<uchar>(row, col) <= 127 &&
                    imgShare.at<uchar>(row, col - 1) > 127)
                {
                    endBlock[counterBlock++] = col;
                    if (counterBlock >= end(endBlock) - begin(endBlock))
                        break;
                }
            }
        }
        if (imgShare.at<uchar>(row, COLSIMAGE - 1) > 127)
        {
            if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                endBlock[counterBlock++] = COLSIMAGE - 1;
        }

        if (flagStartBlock)
        {
            if (row < ROWSIMAGE / 3)
                break;
            if (counterBlock == 0)
            {
                initialStableRows = 0;
                continue;
            }

            // Evaluate both individual blocks and their outer envelope. The
            // envelope represents one road split by an arrow or crosswalk.
            int selectedLeft = -1;
            int selectedRight = -1;
            float bestScore = std::numeric_limits<float>::max();
            auto considerInitial = [&](int left, int right) {
                const int width = right - left;
                if (width <= 0)
                    return;
                int limitWidth = static_cast<int>((COLSIMAGE - (ROWSIMAGE - row)) * 0.65f);
                if (row < ROWSIMAGE * 0.75f)
                    limitWidth = static_cast<int>(COLSIMAGE * 0.5f);
                if (width <= limitWidth)
                    return;
                const int center = (left + right) / 2;
                const int referenceCenter = initialLeft >= 0
                    ? (initialLeft + initialRight) / 2 : COLSIMAGE / 2;
                const int referenceWidth = initialLeft >= 0
                    ? initialRight - initialLeft : width;
                const float score = std::abs(center - referenceCenter) +
                    0.35f * std::abs(width - referenceWidth);
                if (score < bestScore)
                {
                    bestScore = score;
                    selectedLeft = left;
                    selectedRight = right;
                }
            };
            for (int i = 0; i < counterBlock; ++i)
                considerInitial(startBlock[i], endBlock[i]);
            if (counterBlock > 1)
            {
                int largestInternalGap = 0;
                for (int i = 1; i < counterBlock; ++i)
                    largestInternalGap = std::max(
                        largestInternalGap, startBlock[i] - endBlock[i - 1]);
                if (previousInternalGap >= 0 &&
                    largestInternalGap > previousInternalGap + 4)
                    expandingGapRows++;
                else
                    expandingGapRows = std::max(0, expandingGapRows - 1);
                previousInternalGap = largestInternalGap;

                // Arrow holes remain narrow; a fork island widens over rows.
                const bool arrowLikeHole =
                    largestInternalGap <= COLSIMAGE / 8 &&
                    expandingGapRows < 2;
                if (allowOuterEnvelope && arrowLikeHole)
                    considerInitial(startBlock[0], endBlock[counterBlock - 1]);
            }
            else
            {
                previousInternalGap = -1;
                expandingGapRows = 0;
            }

            if (selectedLeft < 0)
            {
                initialStableRows = 0;
                continue;
            }
            const int selectedCenter = (selectedLeft + selectedRight) / 2;
            const int selectedWidth = selectedRight - selectedLeft;
            bool coherent = true;
            if (initialLeft >= 0)
            {
                const int previousCenter = (initialLeft + initialRight) / 2;
                const int previousWidth = initialRight - initialLeft;
                coherent = std::abs(selectedCenter - previousCenter) <= 15 &&
                    std::abs(selectedWidth - previousWidth) <=
                        std::max(1, static_cast<int>(previousWidth * 0.20f));
            }
            initialStableRows = coherent ? initialStableRows + 1 : 1;
            initialLeft = selectedLeft;
            initialRight = selectedRight;

            if (initialStableRows >= 3)
            {
                flagStartBlock = false;
                pointsEdgeLeft.emplace_back(row, selectedLeft);
                pointsEdgeRight.emplace_back(row, selectedRight);
                widthBlock.emplace_back(row, selectedWidth);
                counterSearchRows++;
            }
            spurroadEnable = false;
        }
        else // 其它行色块坐标处理
        {
            if (counterBlock == 0)
            {
                continue; // 跳过缺口行，继续向上搜索
            }

            if (row == 22)
                int a = 0;
            vector<int> indexBlocks;               // 色块序号（行）
            for (int i = 0; i < counterBlock; i++) // 上下行色块的连通性判断
            {
                int g_cover = min(endBlock[i], pointsEdgeRight[pointsEdgeRight.size() - 1].y) -
                              max(startBlock[i], pointsEdgeLeft[pointsEdgeLeft.size() - 1].y);
                if (g_cover >= 0)
                {
                    indexBlocks.push_back(i);
                }
            }

            if (indexBlocks.size() == 0) // 如果没有发现联通色块，跳过缺口继续搜索
            {
                continue;
            }
            else if (indexBlocks.size() == 1) // 只存在单个色块，正常情况，提取边缘信息
            {
                if (endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]] < COLSIMAGE / 20)
                {
                    break;
                }
                pointsEdgeLeft.emplace_back(row, startBlock[indexBlocks[0]]);
                pointsEdgeRight.emplace_back(row, endBlock[indexBlocks[0]]);
                slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1); // 边缘斜率计算
                slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                widthBlock.emplace_back(row, endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]]);
                spurroadEnable = false;
            }
            else if (indexBlocks.size() > 1)
            {
                const int previousLeft = pointsEdgeLeft.back().y;
                const int previousRight = pointsEdgeRight.back().y;
                const int previousCenter = (previousLeft + previousRight) / 2;
                const int previousWidth = previousRight - previousLeft;

                // Keep internal arrow/crosswalk holes from becoming fake outer
                // boundaries by evaluating one coherent road candidate.
                int selectedLeft = COLSIMAGE;
                int selectedRight = 0;
                for (int index : indexBlocks)
                {
                    selectedLeft = std::min(selectedLeft, startBlock[index]);
                    selectedRight = std::max(selectedRight, endBlock[index]);
                }
                int largestInternalGap = 0;
                for (size_t i = 1; i < indexBlocks.size(); ++i)
                    largestInternalGap = std::max(largestInternalGap,
                        startBlock[indexBlocks[i]] - endBlock[indexBlocks[i - 1]]);
                if (previousInternalGap >= 0 &&
                    largestInternalGap > previousInternalGap + 4)
                    expandingGapRows++;
                else
                    expandingGapRows = std::max(0, expandingGapRows - 1);
                previousInternalGap = largestInternalGap;
                const bool allowCoherentEnvelope = allowOuterEnvelope &&
                    largestInternalGap <= COLSIMAGE / 8 &&
                    expandingGapRows < 2;

                const auto candidateAcceptable = [&](int left, int right)
                {
                    const int width = right - left;
                    const int center = (left + right) / 2;
                    const int allowedWidthChange = std::max(8, previousWidth / 4);
                    const int overlap = std::min(right, previousRight) -
                                        std::max(left, previousLeft);
                    return width >= COLSIMAGE / 10 && overlap >= 0 &&
                           std::abs(center - previousCenter) <= 25 &&
                           std::abs(width - previousWidth) <= allowedWidthChange;
                };

                if (!allowCoherentEnvelope ||
                    !candidateAcceptable(selectedLeft, selectedRight))
                {
                    float bestScore = std::numeric_limits<float>::max();
                    int bestIndex = -1;
                    for (int index : indexBlocks)
                    {
                        const int left = startBlock[index];
                        const int right = endBlock[index];
                        const int width = right - left;
                        const int center = (left + right) / 2;
                        const int overlap = std::max(0,
                            std::min(right, previousRight) -
                            std::max(left, previousLeft));
                        if (!candidateAcceptable(left, right))
                            continue;
                        const float score =
                            2.0f * std::abs(center - previousCenter) +
                            std::abs(width - previousWidth) - 3.0f * overlap;
                        if (score < bestScore)
                        {
                            bestScore = score;
                            bestIndex = index;
                        }
                    }
                    if (bestIndex < 0)
                    {
                        pointSpurroad.x = row;
                        pointSpurroad.y = endBlock[indexBlocks.front()];
                        if (!spurroadEnable)
                        {
                            spurroad.push_back(pointSpurroad);
                            spurroadEnable = true;
                        }
                        continue;
                    }
                    selectedLeft = startBlock[bestIndex];
                    selectedRight = endBlock[bestIndex];
                }

                pointsEdgeLeft.emplace_back(row, selectedLeft);
                pointsEdgeRight.emplace_back(row, selectedRight);
                widthBlock.emplace_back(row, selectedRight - selectedLeft);
                slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1);
                slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                counterSearchRows++;

                pointSpurroad.x = row;
                pointSpurroad.y = endBlock[indexBlocks.front()];
                if (!spurroadEnable)
                {
                    spurroad.push_back(pointSpurroad);
                    spurroadEnable = true;
                }
            }
        }
    }

    fillLaneGap();

    stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE); // 计算边缘方差
    stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);

    validRowsCal(); // 有效行计算
    evaluateQuality();
}

/**
 * @brief 填补边线缺口：当某行没有边线点时，用前后两行插值补上
 *        解决施工区出口等位置赛道边线断开导致丢线的问题
 */
void Track::fillLaneGap()
{
    if (!control_algorithms::fillAlignedLaneGaps(
            pointsEdgeLeft, pointsEdgeRight, widthBlock, maxGapRows))
        return;

    for (size_t i = 0; i < pointsEdgeLeft.size(); ++i)
    {
        pointsEdgeLeft[i].slope = 0.0f;
        pointsEdgeRight[i].slope = 0.0f;
        slopeCal(pointsEdgeLeft, static_cast<int>(i));
        slopeCal(pointsEdgeRight, static_cast<int>(i));
    }
}

void Track::evaluateQuality()
{
    quality = LaneQuality{};
    const auto leftReliability = control_algorithms::assessEdgeReliability(
        pointsEdgeLeft, true, COLSIMAGE, ROWSIMAGE, rowCutBottom,
        singleLaneInteriorPointsMin);
    const auto rightReliability = control_algorithms::assessEdgeReliability(
        pointsEdgeRight, false, COLSIMAGE, ROWSIMAGE, rowCutBottom,
        singleLaneInteriorPointsMin);
    quality.leftReliable = leftReliability.reliable;
    quality.rightReliable = rightReliability.reliable;
    quality.leftSingleUsable = leftReliability.singleEdgeUsable;
    quality.rightSingleUsable = rightReliability.singleEdgeUsable;
    quality.leftInteriorPoints = leftReliability.interiorPointCount;
    quality.rightInteriorPoints = rightReliability.interiorPointCount;
    quality.leftBorderRatio = leftReliability.borderRatio;
    quality.rightBorderRatio = rightReliability.borderRatio;
    quality.leftLongestBorderRun = leftReliability.longestBorderRun;
    quality.rightLongestBorderRun = rightReliability.longestBorderRun;
    vector<int> leftByRow(ROWSIMAGE, -1);
    vector<int> rightByRow(ROWSIMAGE, -1);
    for (const auto &point : pointsEdgeLeft)
        if (point.x >= 0 && point.x < ROWSIMAGE)
            leftByRow[point.x] = point.y;
    for (const auto &point : pointsEdgeRight)
        if (point.x >= 0 && point.x < ROWSIMAGE)
            rightByRow[point.x] = point.y;

    vector<float> widths;
    float centerSum = 0.0f;
    float maximumEdgeJump = 0.0f;
    int previousLeft = -1;
    int previousRight = -1;
    int nearestRow = 0;
    for (int row = 0; row < ROWSIMAGE; ++row)
    {
        if (leftByRow[row] < 0 || rightByRow[row] < 0 ||
            rightByRow[row] <= leftByRow[row])
            continue;
        const float width = rightByRow[row] - leftByRow[row];
        widths.push_back(width);
        centerSum += (leftByRow[row] + rightByRow[row]) * 0.5f;
        nearestRow = std::max(nearestRow, row);
        if (previousLeft >= 0)
        {
            maximumEdgeJump = std::max(maximumEdgeJump,
                static_cast<float>(std::abs(leftByRow[row] - previousLeft)));
            maximumEdgeJump = std::max(maximumEdgeJump,
                static_cast<float>(std::abs(rightByRow[row] - previousRight)));
        }
        previousLeft = leftByRow[row];
        previousRight = rightByRow[row];
    }

    quality.commonRows = static_cast<int>(widths.size());
    quality.edgeJump = maximumEdgeJump;
    quality.coversBottom = nearestRow >= ROWSIMAGE - rowCutBottom - 4;
    if (widths.empty())
        return;

    quality.widthMean = std::accumulate(widths.begin(), widths.end(), 0.0f) /
                        widths.size();
    float relativeWidthChange = 0.0f;
    for (size_t i = 1; i < widths.size(); ++i)
        relativeWidthChange += std::abs(widths[i] - widths[i - 1]) /
                               std::max(1.0f, widths[i - 1]);
    quality.widthVariation = widths.size() > 1
        ? relativeWidthChange / (widths.size() - 1)
        : 1.0f;

    const float currentCenter = centerSum / widths.size();
    quality.centerJump = previousCenterValid
        ? std::abs(currentCenter - previousCenter) : 0.0f;
    previousCenter = currentCenter;
    previousCenterValid = true;

    const float coverageScore = std::min(1.0f, quality.commonRows / 50.0f);
    const float widthScore = std::max(0.0f, 1.0f - quality.widthVariation / 0.25f);
    const float centerScore = std::max(0.0f, 1.0f - quality.centerJump / 20.0f);
    const float edgeScore = std::max(0.0f, 1.0f - quality.edgeJump / 40.0f);
    quality.confidence = 0.30f * coverageScore + 0.25f * widthScore +
                         0.20f * centerScore + 0.15f * edgeScore +
                         (quality.coversBottom ? 0.10f : 0.0f);
    quality.valid = quality.leftReliable && quality.rightReliable &&
                    quality.commonRows >= 20 && quality.coversBottom &&
                    quality.widthVariation <= 0.20f &&
                    quality.centerJump <= 15.0f &&
                    quality.edgeJump <= 30.0f &&
                    quality.confidence >= 0.70f;
}
/**
 * @brief 显示赛道线识别结果
 *
 * @param img 需要叠加显示的图像
 */
void Track::drawImage(Mat &img)
{
    for (int i = 0; i < pointsEdgeLeft.size(); i++)
    {
        circle(img, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
               Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < pointsEdgeRight.size(); i++)
    {
        circle(img, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
               Scalar(0, 255, 255), -1); // 黄色点
    }

    for (int i = 0; i < spurroad.size(); i++)
    {
        circle(img, Point(spurroad[i].y, spurroad[i].x), 3,
               Scalar(0, 0, 255), -1); // 红色点
    }

    putText(img, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
            FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 0.8, LINE_AA);
    putText(img, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
            FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 0.8, LINE_AA);
}

/**
 * @brief 边缘斜率计算
 *
 * @param v_edge
 * @param img_height
 * @return double
 */
double Track::stdevEdgeCal(vector<PointX> &v_edge, int img_height)
{
    if (v_edge.size() < img_height / 4)
    {
        return 1000;
    }
    vector<int> v_slope;
    int step = v_edge.size() / 5; // v_edge.size()/10;
    for (int i = step; i < v_edge.size(); i += step)
    {
        if (v_edge[i].x - v_edge[i - step].x)
            v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 / (v_edge[i].x - v_edge[i - step].x));
    }
    if (v_slope.size() > 1)
    {
        double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
        double mean = sum / v_slope.size(); // 均值
        double accum = 0.0;
        for_each(begin(v_slope), end(v_slope), [&](const double d)
                 { accum += (d - mean) * (d - mean); });

        return sqrt(accum / (v_slope.size() - 1)); // 方差
    }
    else
        return 0;
}

/**
 * @brief 边缘斜率计算
 *
 * @param edge
 * @param index
 */
void Track::slopeCal(vector<PointX> &edge, int index)
{
    if (index <= 4)
    {
        return;
    }
    float temp_slop1 = 0.0, temp_slop2 = 0.0;
    if (edge[index].x - edge[index - 2].x != 0)
    {
        temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                     ((edge[index].x - edge[index - 2].x) * 1.0f);
    }
    else
    {
        temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
    }
    if (edge[index].x - edge[index - 4].x != 0)
    {
        temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                     ((edge[index].x - edge[index - 4].x) * 1.0f);
    }
    else
    {
        edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
    }
    if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
    {
        edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
    }
    else if (abs(temp_slop1) != 255)
    {
        edge[index].slope = temp_slop1;
    }
    else
    {
        edge[index].slope = temp_slop2;
    }
}

/**
 * @brief 边缘有效行计算：左/右
 *
 */
void Track::validRowsCal(void)
{
    // 左边有效行
    validRowsLeft = 0;
    if (pointsEdgeLeft.size() > 1)
    {
        for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
            {
                validRowsLeft = i + 1;
                break;
            }
            if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
            {
                validRowsLeft = i + 1;
                break;
            }
        }
    }

    // 右边有效行
    validRowsRight = 0;
    if (pointsEdgeRight.size() > 1)
    {
        for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeRight[i].y <= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
            if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
        }
    }
}

/**
 * @brief 冒泡法求取集合中值
 *
 * @param vec 输入集合
 * @return int 中值
 */
int Track::getMiddleValue(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;
    if (vec.size() == 1)
        return vec[0];

    int len = vec.size();
    while (len > 0)
    {
        bool sort = true; // 是否进行排序操作标志
        for (int i = 0; i < len - 1; ++i)
        {
            if (vec[i] > vec[i + 1])
            {
                swap(vec[i], vec[i + 1]);
                sort = false;
            }
        }
        if (sort) // 排序完成
            break;

        --len;
    }

    return vec[(int)vec.size() / 2];
}
