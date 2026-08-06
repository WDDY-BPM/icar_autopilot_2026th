#pragma once

#include <string>

struct PredictResult
{
    int type{-1};
    std::string label;
    float score{0.0f};
    int x{0};
    int y{0};
    int width{0};
    int height{0};
};

#define LABEL_CONE 0
#define LABEL_PERSON 1
#define LABEL_BUSY 2
#define LABEL_LIMIT 3
#define LABEL_UNLIMIT 4
#define LABEL_PARK 5
#define LABEL_GATE 6
#define LABEL_CROSS 7
#define LABEL_FORK 8
#define LABEL_LEFT 9
#define LABEL_CHOICE 10
#define LABEL_STATION 11
