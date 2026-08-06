#include "ctrl/control_geometry_validator.hpp"

bool validateControlGeometry(const std::vector<PointX> &centerLine,
                             ControlGeometrySource source,
                             GeometryPolicy policy,
                             PathSource pathSource,
                             FsmMode mode)
{
    if (centerLine.empty() || source == ControlGeometrySource::NONE)
        return false;
    if (policy == GeometryPolicy::STOPPED)
        return false;
    if (policy == GeometryPolicy::PLANNED_REQUIRED &&
        (source != ControlGeometrySource::PLANNED ||
         !pathSourceAllowed(pathSource, mode)))
        return false;
    return true;
}
