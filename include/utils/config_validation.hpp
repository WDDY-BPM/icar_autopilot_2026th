#pragma once

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>

template <typename ConfigType>
void validateIcarConfig(const ConfigType &config, int imageRows)
{
    const auto require = [](bool condition, const std::string &message) {
        if (!condition)
            throw std::invalid_argument("[Config] " + message);
    };
    const float speeds[] = {
        config.velLow, config.velHigh, config.velSlow, config.velPark,
        config.velCurve, config.velBusy, config.velStop, config.velCross,
        config.velYfork, config.startupSpeed};
    require(std::all_of(std::begin(speeds), std::end(speeds),
                        [](float speed) { return speed >= 0.0f; }),
            "all speed fields must be >= 0");
    require(config.velHigh >= config.velLow, "velHigh must be >= velLow");
    require(config.servoRate > 0.0f, "servoRate must be > 0");
    require(config.startupServoRate > 0.0f, "startupServoRate must be > 0");
    require(config.startupRampFrames > 0, "startupRampFrames must be > 0");
    require(config.startupStableFrames > 0, "startupStableFrames must be > 0");
    require(config.singleLaneCenterStep > 0, "singleLaneCenterStep must be > 0");
    require(config.singleLaneMaxCenterJump >= config.singleLaneCenterStep,
            "singleLaneMaxCenterJump must be >= singleLaneCenterStep");
    require(config.rowCutUp < imageRows && config.rowCutBottom < imageRows &&
                config.rowCutUp + config.rowCutBottom < imageRows,
            "rowCutUp/rowCutBottom exceed image height");
    require(config.score >= 0.0f && config.score <= 1.0f,
            "score must be in [0,1]");
    require(config.totalLaps >= 1 && config.totalLaps <= 3,
            "totalLaps must be in [1,3]");

    const auto validateLap = [&](const auto &lap, int number) {
        const std::string prefix = "lap" + std::to_string(number) + " invalid: ";
        const int primaryPaths = static_cast<int>(lap.park) +
            static_cast<int>(lap.busy) + static_cast<int>(lap.yfork) +
            static_cast<int>(lap.fork);
        require(primaryPaths <= 1, prefix +
            "park, busy, yfork and fork are mutually exclusive");
        require(lap.park ? lap.parkSpot >= 1 && lap.parkSpot <= 4
                         : lap.parkSpot == 0,
            prefix + "parkSpot must be 1..4 only when park=true");
        require(lap.busyStopPoint >= 0 && lap.busyStopPoint <= 2,
            prefix + "busyStopPoint must be in [0,2]");
        require(!lap.busyStopEnable || (lap.busy && lap.station),
            prefix + "busyStopEnable requires busy=true and station=true");
        require(!lap.manualTakeover || lap.busy,
            prefix + "manualTakeover requires busy=true");
        require(lap.yfork || !lap.yforkLeft,
            prefix + "yforkLeft requires yfork=true");
    };
    validateLap(config.lap1, 1);
    validateLap(config.lap2, 2);
    validateLap(config.lap3, 3);
}
