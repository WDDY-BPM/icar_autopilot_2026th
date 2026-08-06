#include "config/config_loader.hpp"

#include <fstream>
#include <stdexcept>

Config::LapConfig parseLapConfig(const nlohmann::json &json)
{
    Config::LapConfig lap;
    lap.fork = json.value("fork", false);
    lap.park = json.value("park", false);
    lap.parkSpot = json.value("parkSpot", 0);
    lap.busy = json.value("busy", false);
    lap.slow = json.value("slow", false);
    lap.stop = json.value("stop", false);
    lap.cross = json.value("cross", true);
    lap.yfork = json.value("yfork", false);
    lap.yforkLeft = lap.yfork && json.value("yforkLeft", false);
    lap.station = json.value("station", true);
    lap.obstacle = json.value("obstacle", true);
    lap.manualTakeover = json.value("manualTakeover", false);
    lap.busyStopEnable = json.value("busyStopEnable", false);
    lap.busyStopPoint = json.value("busyStopPoint", 0);
    return lap;
}

Config loadConfig(const std::string &path)
{
    std::ifstream file(path);
    if (!file.good())
        throw std::runtime_error("[Config] file not found: " + path);
    nlohmann::json configs;
    try
    {
        file >> configs;
        const auto common = configs.at("通用配置参数");
        Config config;
        config.velLow = common.at("velLow");
        config.velHigh = common.at("velHigh");
        config.velSlow = common.at("velSlow");
        config.velPark = common.at("velPark");
        config.velCurve = common.at("velCurve");
        config.velBusy = common.at("velBusy");
        config.velStop = common.at("velStop");
        config.velCross = common.at("velCross");
        config.velYfork = common.value("velYfork", 0.7f);
        config.runP1 = common.at("runP1");
        config.runP2 = common.at("runP2");
        config.turnD = common.at("turnD");
        config.laneHeadingGain = common.value("laneHeadingGain", 300.0f);
        config.laneHeadingMaxCorrection = common.value("laneHeadingMaxCorrection", 60.0f);
        config.laneHeadingFadeError = common.value("laneHeadingFadeError", 40.0f);
        config.singleLaneHeadingConfidence = common.value("singleLaneHeadingConfidence", 0.45f);
        config.borderClippedHeadingConfidence = common.value("borderClippedHeadingConfidence", 0.35f);
        config.parkingHeadingConfidence = common.value("parkingHeadingConfidence", 0.65f);
        config.singleLaneInteriorPointsMin = common.value("singleLaneInteriorPointsMin", 12);
        config.singleLaneMaxCenterJump = common.value("singleLaneMaxCenterJump", 45);
        config.singleLaneCenterStep = common.value("singleLaneCenterStep", 8);
        config.steeringFilterTau = common.value("steeringFilterTau", 0.065f);
        config.maxErrorRate = common.value("maxErrorRate", 360.0f);
        config.servoRate = common.value("servoRate", 600.0f);
        config.startupServoRate = common.value("startupServoRate", 550.0f);
        config.startupServoLimit = common.value("startupServoLimit", 180);
        config.startupStableFrames = common.value("startupStableFrames", 12);
        config.startupRampFrames = common.value("startupRampFrames", 60);
        config.startupSpeed = common.value("startupSpeed", 0.10f);
        config.maxGapRows = common.value("maxGapRows", 8);
        config.debug = common.value("debug", false);
        config.saveImg = common.value("saveImg", false);
        config.saveIpm = common.value("saveIpm", false);
        config.rowCutUp = common.at("rowCutUp");
        config.rowCutBottom = common.at("rowCutBottom");
        config.overlap = common.at("overlap");
        config.score = common.at("score");
        config.binary = common.at("binary");
        config.model = common.at("model");
        config.video = common.at("video");
        config.alertTarget = common.value("alertTarget", "none");
        config.requireStartCone = common.value("requireStartCone", true);
        config.totalLaps = configs.at("圈数配置").at("totalLaps");
        const auto laps = configs.at("每圈功能使能配置");
        config.lap1 = parseLapConfig(laps.at("lap1"));
        config.lap2 = parseLapConfig(laps.at("lap2"));
        config.lap3 = parseLapConfig(laps.at("lap3"));
        return config;
    }
    catch (const nlohmann::detail::exception &error)
    {
        throw std::runtime_error(std::string("[Config] JSON parse failed: ") + error.what());
    }
}
