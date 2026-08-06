#pragma once

#include <cstdint>
#include <string>
#include "utils/json.hpp"

struct Config
{
    float velLow = 1.3f;
    float velHigh = 1.3f;
    float velSlow = 0.5f;
    float velPark = 0.6f;
    float velCurve = 1.0f;
    float velBusy = 0.8f;
    float velStop = 0.7f;
    float velCross = 0.7f;
    float velYfork = 0.7f;
    float runP1 = 2.2f;
    float runP2 = 0.007f;
    float turnD = 0.027f;
    float laneHeadingGain = 300.0f;
    float laneHeadingMaxCorrection = 60.0f;
    float laneHeadingFadeError = 40.0f;
    float singleLaneHeadingConfidence = 0.45f;
    float borderClippedHeadingConfidence = 0.35f;
    float parkingHeadingConfidence = 0.65f;
    int singleLaneInteriorPointsMin = 12;
    int singleLaneMaxCenterJump = 45;
    int singleLaneCenterStep = 8;
    float steeringFilterTau = 0.065f;
    float maxErrorRate = 360.0f;
    float servoRate = 600.0f;
    float startupServoRate = 550.0f;
    int startupServoLimit = 180;
    int startupStableFrames = 12;
    int startupRampFrames = 60;
    float startupSpeed = 0.10f;
    int maxGapRows = 8;
    bool debug = false;
    bool saveImg = false;
    bool saveIpm = false;
    uint16_t rowCutUp = 40;
    uint16_t rowCutBottom = 20;
    float overlap = 0.3f;
    float score = 0.2f;
    int binary = -1;
    std::string model = "../res/models/yolov3_mobilenet_v1";
    std::string video = "../res/samples/sample.mp4";
    std::string alertTarget = "none";
    bool requireStartCone = true;
    int totalLaps = 3;

    struct LapConfig
    {
        bool fork = false;
        bool park = false;
        bool busy = false;
        bool slow = false;
        bool stop = false;
        bool cross = true;
        bool yfork = false;
        bool yforkLeft = false;
        bool station = true;
        bool obstacle = true;
        int parkSpot = 0;
        bool manualTakeover = false;
        bool busyStopEnable = false;
        int busyStopPoint = 0;
    };

    LapConfig lap1;
    LapConfig lap2;
    LapConfig lap3;
    LapConfig *currentLapConfig = nullptr;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, velLow, velHigh, velSlow, velPark,
        velCurve, velBusy, velStop, velCross, velYfork, runP1, runP2, turnD,
        laneHeadingGain, laneHeadingMaxCorrection, laneHeadingFadeError,
        singleLaneHeadingConfidence, borderClippedHeadingConfidence,
        parkingHeadingConfidence, singleLaneInteriorPointsMin,
        singleLaneMaxCenterJump, singleLaneCenterStep, steeringFilterTau,
        maxErrorRate, servoRate, startupServoRate, startupServoLimit,
        startupStableFrames, startupRampFrames, startupSpeed, maxGapRows, debug,
        saveImg, saveIpm, rowCutUp, rowCutBottom, overlap, score, binary, model,
        video, alertTarget, totalLaps);
};
