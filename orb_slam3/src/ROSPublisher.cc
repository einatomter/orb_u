#include "ROSPublisher.h"

ROSPublisher::ROSPublisher():
    node_name("/orb_slam3") 
{
    ros::NodeHandle nh;
    publisher = nh.advertise<std_msgs::Int32>(node_name + "/matched_inliers", 10);
    publisher2 = nh.advertise<std_msgs::Int32>(node_name + "/features_inliers", 10);
    publisher3 = nh.advertise<orb_slam3_ros::MapInfo>(node_name + "/map_info", 2);
    publisher4 = nh.advertise<std_msgs::Int32>(node_name + "/loop_closing", 10);
    is_running = true;
    publisher_thread = std::thread(&ROSPublisher::publishLoop, this);
}

ROSPublisher::~ROSPublisher() {
    is_running = false;
    if (publisher_thread.joinable()) {
        publisher_thread.join();
    }
}


// Tracking
void ROSPublisher::publishInliers(int data) {
    std::unique_lock<std::mutex> lock(data_mutex);
    data_queue.push(data);
}

void ROSPublisher::setTrackedFeatures(int data) {
    std_msgs::Int32 msg;
    msg.data = data;
    publisher2.publish(msg);
}


// Mapping
void ROSPublisher::setMapId(int mapId, int initStep, double timeStamp, double scale)
{
    orb_slam3_ros::MapInfo msg;
    msg.map_id = mapId;
    msg.init_step = initStep;
    msg.scale = scale;
    msg.timestamp = timeStamp;

    mMapInformation.mMapId = mapId;
    mMapInformation.mInitStep = initStep;
    mMapInformation.scale = scale;
    mMapInformation.mTimestamp = timeStamp;


    // std::cout << std::fixed;
    // std::cout << "mMapId: " << mMapInformation.mMapId << " " << 
    //              "Procedure: " << mMapInformation.mInitStep << " " << 
    //              "Timestamp: " << mMapInformation.mTimestamp << std::endl;
    // std::cout << std::defaultfloat;

    publisher3.publish(msg);
}

void ROSPublisher::setLoopClosingInfo(int data) {


    std::cout << "BoW matches: " << data << std::endl;

    std_msgs::Int32 msg;
    msg.data = data;
    publisher4.publish(msg);
}


void ROSPublisher::publishMessage(int data) {
    std_msgs::Int32 msg;
    msg.data = data;
    publisher.publish(msg);
}

void ROSPublisher::publishLoop() {
    ros::Rate loop_rate(50);

    while (ros::ok() && is_running) {

        std::unique_lock<std::mutex> lock(data_mutex);

        if (!data_queue.empty()) 
        {
            int data = data_queue.front();
            data_queue.pop();
            
            // Publish the data
            publishMessage(data);
        }

        lock.unlock();

        loop_rate.sleep();
    }
}