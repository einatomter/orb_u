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

struct MapInformation {
    int mMapId;
    int mInitStep;
    double mTimestamp;
};


class ROSPublisher {
public:
    ROSPublisher();
    ~ROSPublisher();

    // Tracker
    void publishInliers(int data);
    void setTrackedFeatures(int data);

    // Localmapper
    void setMapId(int mapId, int initStep, double timeStamp);
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

    // Tracker variables
    std::mutex data_mutex;
    std::queue<int> data_queue;

    // Localmapper variables
    MapInformation mMapInformation;
};

#endif // ROSPUBLISHER_H
