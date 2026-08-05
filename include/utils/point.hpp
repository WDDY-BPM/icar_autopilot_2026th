#pragma once

struct PointX
{
    int x = 0;
    int y = 0;
    float slope = 0.0f;

    PointX() = default;
    PointX(int row, int column) : x(row), y(column) {}
    PointX(int row, int column, float edgeSlope)
        : x(row), y(column), slope(edgeSlope) {}
};
