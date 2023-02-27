// ---------------------------------------
// PRESSURE FUNCTIONS
// ---------------------------------------

/*
    NOTES:
        depth needs to be measured relative to first depth measurement
        function required for when sensor readings are greater than camera readings
            get depth average
            get new measurement noise
        initialization procedure: scale needs to be adjusted according to pressure sensor readings

    ASSUMPTIONS:
        ROV always starts with z axis aligned with world reference frame
*/

#ifndef PRESSURETYPES_H
#define PRESSURETYPES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include <Thirdparty/g2o/g2o/types/sim3.h>

// #include "G2oTypes.h"

#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace UW
{

// constant values
const float PRESSURE_SEA_LEVEL  = 101.325;  // kPa
const float FLUID_DENSITY       = 1024.0;   // kg/m3
const float GRAVITY_VALUE       = 9.81;     // m/s2
// sensor noise
// roughly 1kPa given the operating range of the ROV
// 3kPa if based on datasheet
const float SENSOR_NOISE = 1.0;
// calculated depth noise based on sensor noise (m)
const float DEPTH_NOISE = SENSOR_NOISE * 1000 / (FLUID_DENSITY * GRAVITY_VALUE);

const Eigen::Vector3d defaultDepthAxis(0, 1, 0);

// object for containing pressure and depth values
class Point
{
public:
    Point();
    // Copy Constructor
    Point(const Point &point);
    Point(const float &inputValue, const Eigen::Vector3d &inputDepthAxis = defaultDepthAxis, const bool &usePressure = false);

    // converts pressure value (kPa) to depth value (m)
    // direction is also switched (up-positive down-negative)
    float pressureToDepth(float fluidPressure);
    // returns depth relative to initialized depth value
    float relativeDepthHeight();

    // set initial depth value
    void setInitDepth(float newInitDepth);

    // raw measurement reading (kPa)
    // up-negative down-positive
    float pressure = 0;

    // initial depth upon SLAM initialization (m)
    // up-negative down-positive
    float initDepth = 0;

    // current depth (m)
    // up-negative down-positive
    float depth = 0;

    Eigen::Vector3d depthAxis;

private:
    // UNUSED
    // converts depth value to be aligned with body frame (up-positive down-negative)
    float depthToDepthHeight(float depth);
};

} // namespace UW

#endif // PRESSURETYPES_H
