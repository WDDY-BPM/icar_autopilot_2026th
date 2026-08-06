#pragma once

#include <string>
#include "config/config.hpp"

Config loadConfig(const std::string &path);
Config::LapConfig parseLapConfig(const nlohmann::json &json);
