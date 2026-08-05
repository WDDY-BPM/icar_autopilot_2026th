#pragma once

#include <iostream>

#define CHECK(...)                                                       \
    do                                                                   \
    {                                                                    \
        if (!(__VA_ARGS__))                                              \
        {                                                                \
            std::cerr << "CHECK failed: " << #__VA_ARGS__               \
                      << " at " << __FILE__ << ":" << __LINE__          \
                      << std::endl;                                      \
            return 1;                                                    \
        }                                                                \
    } while (false)
