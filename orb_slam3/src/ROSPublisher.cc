#include "ROSPublisher.h"

ROSPublisher::ROSPublisher():
    mRosNodeName("/orb_slam3/analysis") 
{
    ros::NodeHandle nh;
    mRosPInliers = nh.advertise<std_msgs::Int32>(mRosNodeName + "/matched_inliers", 10);
    mRosPMapInfo = nh.advertise<orb_slam3_ros::MapInfo>(mRosNodeName + "/map_info", 2);
    mRosPLoopClosing = nh.advertise<orb_slam3_ros::LoopClosingInfo>(mRosNodeName + "/loop_closing", 10);
    bIsRunning = true;
    mtPublisherThread = std::thread(&ROSPublisher::Run, this);
}

ROSPublisher::~ROSPublisher() {
    bIsRunning = false;
    if (mtPublisherThread.joinable()) {
        mtPublisherThread.join();
    }
}


// Tracking
void ROSPublisher::SetInliers(int data) {
    std::unique_lock<std::mutex> lock(mMutexInliers);
    mqInliers.push(data);
}


void ROSPublisher::PublishInliers() {

    std::unique_lock<std::mutex> lock(mMutexInliers);

    if (!mqInliers.empty()) 
    {
        int data = mqInliers.front();
        mqInliers.pop();
        
        std_msgs::Int32 msg;
        msg.data = data;
        mRosPInliers.publish(msg);
    }
}

// Mapping
void ROSPublisher::SetMapInitInfo(int mapId, int initStep, double timeStamp, double scale)
{
    std::unique_lock<std::mutex> lock(mMutexMapInitInfo);


    MapInformation mapInitInfo;

    mapInitInfo.mapId = mapId;
    mapInitInfo.initStep = initStep;
    mapInitInfo.scale = scale;
    mapInitInfo.timestamp = timeStamp;

    mqMapInitInfo.push(mapInitInfo);
}


void ROSPublisher::PublishMapInitInfo() {
    std::unique_lock<std::mutex> lock(mMutexMapInitInfo);

    if (!mqMapInitInfo.empty()) 
    {
        MapInformation data = mqMapInitInfo.front();
        mqMapInitInfo.pop();

        orb_slam3_ros::MapInfo msg;
        msg.map_id = data.mapId;
        msg.init_step = data.initStep;
        msg.scale = data.scale;
        msg.timestamp = data.timestamp;
        
        // Publish the data
        mRosPMapInfo.publish(msg);

        
        // std::cout << std::fixed;
        // std::cout << "mMapId: " << msg.map_id << " " << 
        //              "Procedure: " << msg.init_step << " " << 
        //              "Scale: " << msg.scale << " " <<
        //              "Timestamp: " << msg.timestamp << std::endl;
        // std::cout << std::defaultfloat;
    }
}

// Loopclosing

void ROSPublisher::SetLoopClosingInfo(int numBoWMatches, int numMatches, int numProjMatches, 
                                      int numOptMatches, int numProjOptMatches, int numKFs)
{
    std::unique_lock<std::mutex> lock(mMutexLoopClosingInfo);

    LoopClosingInformation loopClosingInfo;

    loopClosingInfo.numBoWMatches = numBoWMatches;
    loopClosingInfo.numMatches = numMatches;
    loopClosingInfo.numProjMatches = numProjMatches;
    loopClosingInfo.numOptMatches = numOptMatches;
    loopClosingInfo.numProjOptMatches = numProjOptMatches;
    loopClosingInfo.numKFs = numKFs;

    mqLoopClosingInfo.push(loopClosingInfo);
}

void ROSPublisher::PublishLoopClosingInfo() {
    std::unique_lock<std::mutex> lock(mMutexLoopClosingInfo);

    if (!mqLoopClosingInfo.empty()) 
    {
        LoopClosingInformation data = mqLoopClosingInfo.front();
        mqLoopClosingInfo.pop();

        orb_slam3_ros::LoopClosingInfo msg;
        msg.num_bow_matches = data.numBoWMatches;
        msg.num_matches = data.numMatches;
        msg.num_proj_matches = data.numProjMatches;
        msg.num_opt_matches = data.numOptMatches;
        msg.num_proj_opt_matches = data.numProjOptMatches;
        msg.num_kfs = data.numKFs;
        
        // Publish the data
        mRosPLoopClosing.publish(msg);

        // std::cout << msg.num_bow_matches << " " << msg.num_matches << " " << msg.num_proj_matches << " " << 
        //              msg.num_opt_matches << " " << msg.num_proj_opt_matches << " " << msg.num_kfs << std::endl;
    }
}

void ROSPublisher::Run() {
    ros::Rate loop_rate(50);

    while (ros::ok() && bIsRunning) {

        PublishInliers();
        PublishMapInitInfo();
        PublishLoopClosingInfo();

        loop_rate.sleep();
    }
}