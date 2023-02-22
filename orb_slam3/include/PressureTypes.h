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


class EdgeDepth2: public g2o::BaseUnaryEdge<1, float, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeDepth2(){}

    // Set camera orientation relative to depth axis
    void setDepthAxis(Eigen::Vector3d& input){
        _depthAxis = input;
    }

    void computeError(){
        const g2o::VertexSE3Expmap* VPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        // VPose->estimate().rotation().toRotationMatrix();
        Eigen::Vector3d translation(VPose->estimate().translation());
        const Eigen::Matrix<double, 1, 1> est(translation.transpose() * _depthAxis);
        const Eigen::Matrix<double, 1, 1> obs(_measurement);
        _error = (obs - est); 
    }

    // redefined to 1D definition ((x-mu)^2/sigma^2)
    virtual double chi2() const{
        // information is already inverse and can therefore be multiplied directly
        // std::cout << "chi2 depth: " << pow(_error(0,0)*information()(0,0), 2) << std::endl;
        return pow(_error(0,0)*information()(0,0), 2);
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

private:
    Eigen::Vector3d _depthAxis;
};


// scale vertex
class VertexScale : public g2o::BaseVertex<1,double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale(){
        setEstimate(1.0);
    }
    VertexScale(double ps){
        setEstimate(ps);
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl(){
        setEstimate(1.0);
    }

    virtual void oplusImpl(const double *update_){
        setEstimate(estimate() + *update_);
    }
};

class EdgeScale: public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3Expmap, VertexScale>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeScale(){}

    void setDepthAxis(Eigen::Vector3d& input){
        _depthAxis = input;
    }

    void setTranslation(const Eigen::Vector3d& input)
    {
        _translation = input;
    }

    void computeError(){
        const g2o::VertexSE3Expmap* VPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const VertexScale* VScale = static_cast<const VertexScale*>(_vertices[1]);
        Eigen::Vector3d translation(VPose->estimate().translation());
        
        const Eigen::Matrix<double, 1, 1> est(VScale->estimate() * (translation.transpose() * _depthAxis));
        const Eigen::Matrix<double, 1, 1> obs(_measurement);
        _error = (obs - est);
    }

    // redefined to 1D definition ((x-mu)^2/sigma^2)
    virtual double chi2() const{
        // information is already inverse and can therefore be multiplied directly
        // std::cout << "chi2 depth: " << pow(_error(0,0)*information()(0,0), 2) << std::endl;
        return pow(_error(0,0)*information()(0,0), 2);
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

private:
    Eigen::Vector3d _depthAxis;
    Eigen::Vector3d _translation;
};

} // namespace UW

#endif // PRESSURETYPES_H
