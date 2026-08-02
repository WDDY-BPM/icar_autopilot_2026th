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
 * @file predeal.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 图像预处理
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ctrl/predeal.hpp"

using namespace cv;
using namespace std;

/**
 * @brief 图像矫正参数初始化
 *
 * @param bin 二值化阈值
 */
Predeal::Predeal(int bin)
    : binary(bin)
{
    // 读取xml中的相机标定参数
    cameraMatrix = cv::Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
    distCoeffs = cv::Mat(1, 5, CV_32FC1, Scalar::all(0));   // 相机的畸变矩阵
    FileStorage file;
    if (file.open("../res/calibration/valid/calibration.xml", FileStorage::READ)) // 读取本地保存的标定文件
    {
        file["cameraMatrix"] >> cameraMatrix;
        file["distCoeffs"] >> distCoeffs;
        cout << "相机矫正参数初始化成功!" << endl;
        enable = true;
    }
    else
    {
        cout << "打开相机矫正参数失败!!!" << endl;
        enable = false;
    }
}

/**
 * @brief 图像二值化
 *
 * @param img  输入图像
 * @return cv::Mat 二值化图像
 */
cv::Mat Predeal::binaryzation(cv::Mat &img)
{
    cv::Mat imgGray, blurred, imgBin;
    cvtColor(img, imgGray, COLOR_BGR2GRAY);
    GaussianBlur(imgGray, blurred, Size(5, 5), 0.0);

    int thresholdValue = binary;
    if (binary < 0)
    {
        // Estimate illumination from the road-dominant lower/central ROI, then
        // smooth it over time so reflections cannot change the whole mask in
        // one frame. Camera exposure/calibration settings remain untouched.
        const int roiX = blurred.cols / 8;
        const int roiY = blurred.rows / 3;
        const Rect roadRoi(roiX, roiY,
                           blurred.cols - 2 * roiX,
                           blurred.rows - roiY);
        cv::Mat unused;
        const float currentThreshold = static_cast<float>(threshold(
            blurred(roadRoi), unused, 0, 255, THRESH_BINARY | THRESH_OTSU));
        filteredThreshold = filteredThreshold < 0.0f
            ? currentThreshold
            : 0.85f * filteredThreshold + 0.15f * currentThreshold;
        thresholdValue = static_cast<int>(std::lround(filteredThreshold));
    }
    else
    {
        thresholdValue = std::clamp(binary, 0, 255);
    }

    threshold(blurred, imgBin, thresholdValue, 255, THRESH_BINARY);
    cv::Mat imgInv;
    bitwise_not(imgBin, imgInv);
    morphologyEx(imgInv, imgInv, MORPH_CLOSE,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));
    return imgInv;
}
/**
 * @brief 矫正图像
 *
 * @param img 输入图像
 * @return cv::Mat 输出图像
 */
void Predeal::correction(cv::Mat &img)
{
    if (enable)
    {
        Size sizeImage; // 图像的尺寸
        sizeImage.width = img.cols;
        sizeImage.height = img.rows;

        cv::Mat mapx = cv::Mat(sizeImage, CV_32FC1);    // 经过矫正后的X坐标重映射参数
        cv::Mat mapy = cv::Mat(sizeImage, CV_32FC1);    // 经过矫正后的Y坐标重映射参数
        cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

        // 采用initUndistortRectifyMap+remap进行图像矫正
        initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
        remap(img, img, mapx, mapy, INTER_LINEAR);
    }
}

/**
 * @brief 图像裁剪
 *
 * @param img
 */
void Predeal::imgCutting(cv::Mat &img)
{
    // 图像裁剪
    // 提取 55~175 行，列方向为 30~310
    cv::Rect roi(40, 55, 260, 120); // (x, y, width, height)
    img = img(roi);                 // 抠图

    // 纵向拉伸到300x180（height=300, width=180）
    cv::resize(img, img, cv::Size(300, 180), 0, 0, cv::INTER_NEAREST);
}
