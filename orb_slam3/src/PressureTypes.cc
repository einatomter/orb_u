/**
* Functions
*/

#include <iostream>
#include "PressureTypes.h"

namespace UW
{

Point::Point():
    pressure(PRESSURE_SEA_LEVEL)
{
    depth = pressureToDepth(pressure);
    initDepth = depth;
}

// Copy Constructor
Point::Point(const Point &point):
    pressure(point.pressure), 
    initDepth(point.initDepth), 
    depth(point.depth), 
    depthAxis(point.depthAxis)
{}

Point::Point(const float &inputValue, const Eigen::Vector3d &inputDepthAxis, const bool &usePressure)
{
    if (usePressure)
    {
        pressure = inputValue;
        depth = pressureToDepth(pressure);

    }
    else
    {
        depth = inputValue;
        // TODO: either do pressure calculation or remove pressure variable
        pressure = -1;
    }

    depthAxis = inputDepthAxis;
}

float Point::pressureToDepth(float fluidPressure)
{
    return (fluidPressure - PRESSURE_SEA_LEVEL)*1000 / (FLUID_DENSITY * GRAVITY_VALUE);
}

float Point::relativeDepthHeight()
{
    return depth - initDepth;
}

void Point::setInitDepth(float newInitDepth)
{
    initDepth = newInitDepth;
}

float Point::depthToDepthHeight(float depth)
{
    return -depth;
}
} // namespace UW