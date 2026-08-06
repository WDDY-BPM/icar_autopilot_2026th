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
 * @file track.hpp
 * @author Leo (liaotengjun@saishukeji.com)
 * @brief 车道线检测
 * @version 0.1
 * @date 2025-07-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include "utils/tools.hpp"
#include "ctrl/control_algorithms.hpp"

using namespace cv;
using namespace std;

class Track
{
public:
    struct LaneQuality
    {
        bool valid = false;
        int commonRows = 0;
        float widthMean = 0.0f;
        float widthVariation = 1.0f;
        float centerJump = 0.0f;
        float edgeJump = 0.0f;
        float confidence = 0.0f;
        bool coversBottom = false;
        bool leftReliable = false;
        bool rightReliable = false;
        bool leftSingleUsable = false;
        bool rightSingleUsable = false;
        bool leftClipped = false;  // 左边缘被图像左边界裁剪
        bool rightClipped = false; // 右边缘被图像右边界裁剪
        int leftInteriorPoints = 0;
        int rightInteriorPoints = 0;
        float leftBorderRatio = 1.0f;
        float rightBorderRatio = 1.0f;
        int leftLongestBorderRun = 0;
        int rightLongestBorderRun = 0;
    };
    vector<PointX> pointsEdgeLeft;  // 赛道左边缘点集
    vector<PointX> pointsEdgeRight; // 赛道右边缘点集
    vector<PointX> widthBlock;      // 色块宽度=终-起（每行）
    vector<PointX> spurroad;        // 保存岔路信息
    double stdevLeft;               // 边缘斜率方差（左）
    double stdevRight;              // 边缘斜率方差（右）
    int validRowsLeft = 0;          // 边缘有效行数（左）
    int validRowsRight = 0;         // 边缘有效行数（右）
    uint16_t rowCutUp = 1;          // 图像顶部切行
    uint16_t rowCutBottom = 20;
    int maxGapRows = 8;
    int singleLaneInteriorPointsMin = 12;
    bool allowOuterEnvelope = true; // Disabled while an AI fork marker is active
    LaneQuality quality;

    void handle(Mat img);
    void handle(bool isResearch, uint16_t rowStart);
    void drawImage(Mat &img);
    double stdevEdgeCal(vector<PointX> &v_edge, int img_height);
    void fillLaneGap(); // Jointly fill short gaps in both edges and width data

private:
    Mat imgShare; // 赛道搜索图像
    void slopeCal(vector<PointX> &edge, int index);
    void validRowsCal(void);
    int getMiddleValue(vector<int> vec);
    void evaluateQuality();
    float previousCenter = COLSIMAGE / 2.0f;
    bool previousCenterValid = false;
};
