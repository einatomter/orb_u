#ifndef ROSPUBLISHER_H
#define ROSPUBLISHER_H

#include <thread>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <orb_slam3_ros/TrackingInfo.h>
#include <orb_slam3_ros/MapInfo.h>
#include <orb_slam3_ros/LoopClosingInfo.h>

struct TrackingInformation {
    int trackedPoints;
    int inliers;
};

struct MapInformation {
    int mapId;
    int initStep;
    double scale;
    double timestamp;
};

struct LoopClosingInformation {
    int numBoWMatches;
    int numMatches;
    int numProjMatches;
    int numOptMatches;
    int numProjOptMatches;
    int numKFs;
};


class ROSPublisher {
public:
    ROSPublisher();
    ~ROSPublisher();

    void PublishTimeMS(double time_ms);

    // Tracking
    void SetTrackingInfo(int trackedPoints, int inliers);

    // Mapping
    void SetMapInitInfo(int mapId, int initStep, double timeStamp, double scale = 1.0);

    // Loopclosing
    void SetLoopClosingInfo(int numBoWMatches, int numMatches, int numProjMatches, 
                            int numOptMatches, int numProjOptMatches, int numKFs);

private:
    void PublishInliers();
    void PublishMapInitInfo();
    void PublishLoopClosingInfo();
    void Run();

    // thread variables
    std::thread mtPublisherThread;
    bool bIsRunning;

    // ROS variables
    std::string mRosNodeName;
    ros::Publisher mRosPTrackingInfo;
    ros::Publisher mRosPMapInitInfo;
    ros::Publisher mRosPLoopClosingInfo;
    ros::Publisher mRosPTimeMS;

    // Tracking
    std::mutex mMutexInliers;
    std::queue<TrackingInformation> mqInliers;

    // Mapping
    std::mutex mMutexMapInitInfo;
    std::queue<MapInformation> mqMapInitInfo;

    // Loopclosing
    std::mutex mMutexLoopClosingInfo;
    std::queue<LoopClosingInformation> mqLoopClosingInfo;
};

#endif // ROSPUBLISHER_H
