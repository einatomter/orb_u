/**
* Functions
*/

#include <iostream>
#include "PressureTypes.h"

namespace UW
{

Point::Point(const float &pressureReading)
{
    pressure = pressureReading;
    depth = pressureToDepth(pressure);

}

float Point::pressureToDepth(float fluidPressure)
{
    return -(fluidPressure - PRESSURE_SEA_LEVEL)*1000 / (FLUID_DENSITY * GRAVITY_VALUE);
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