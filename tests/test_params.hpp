#pragma once

#include "utils/params.hpp"

#include <memory>

#ifndef ICAR_TEST_RES_CONFIG
#error "ICAR_TEST_RES_CONFIG must point at res/config.json for logic tests"
#endif

inline std::shared_ptr<Params> makeTestParams()
{
    return std::shared_ptr<Params>(new Params(ICAR_TEST_RES_CONFIG));
}

inline void setStraightTrack(std::shared_ptr<Params> &params,
                             int startRow, int endRow,
                             int leftColumn, int rightColumn)
{
    params->track->pointsEdgeLeft.clear();
    params->track->pointsEdgeRight.clear();
    for (int row = startRow; row >= endRow; --row)
    {
        params->track->pointsEdgeLeft.emplace_back(row, leftColumn);
        params->track->pointsEdgeRight.emplace_back(row, rightColumn);
    }
}
