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

#include <orb_slam3_ros/MapInfo.h>

struct MapInformation {
    int mMapId;
    int mInitStep;
    double scale;
    double mTimestamp;
};


class ROSPublisher {
public:
    ROSPublisher();
    ~ROSPublisher();

    // Tracking
    void publishInliers(int data);
    void setTrackedFeatures(int data);

    // Mapping
    void setMapId(int mapId, int initStep, double timeStamp, double scale = 1.0);

    // Loopclosing
    void setLoopClosingInfo(int data);

private:
    void publishMessage(int data);
    void publishLoop();

    // thread variables
    std::thread publisher_thread;
    bool is_running;

    // ROS variables
    std::string node_name;
    ros::Publisher publisher;
    ros::Publisher publisher2;
    ros::Publisher publisher3;
    ros::Publisher publisher4;

    // Tracking
    std::mutex data_mutex;
    std::queue<int> data_queue;

    // Mapping
    MapInformation mMapInformation;

    // Loopclosing
};

#endif // ROSPUBLISHER_H
