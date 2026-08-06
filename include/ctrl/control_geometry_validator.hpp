#pragma once

#include <vector>
#include "ctrl/control_geometry.hpp"
#include "runtime/fsm_mode.hpp"
#include "runtime/path_override.hpp"

bool validateControlGeometry(const std::vector<PointX> &centerLine,
                             ControlGeometrySource source,
                             GeometryPolicy policy,
                             PathSource pathSource,
                             FsmMode mode);
