/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include "common.h"

using namespace std;

class PressureGrabber
{
public:
    PressureGrabber(){};
    void GrabPressure(const sensor_msgs::FluidPressureConstPtr &pressure_msg);

    queue<sensor_msgs::FluidPressureConstPtr> pBuf;
    sensor_msgs::FluidPressure bufLastVal;
    float pressure = 0;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(PressureGrabber *pPGb, double timeshift_cam_imu): 
        mpPGb(pPGb), mTimeshift_cam_imu(timeshift_cam_imu){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithPressure();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
    PressureGrabber *mpPGb;
    double mTimeshift_cam_imu;
};

//////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_UW");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    // TODO: check for pressure frame id
    // node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");
    double timeshift_cam_imu;
    node_handler.param<double>(node_name + "/timeshift_cam_imu", timeshift_cam_imu, 0.0);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin, true);

    // ImuGrabber imugb;
    PressureGrabber pgb;
    ImageGrabber igb(&pgb, timeshift_cam_imu);

    ros::Subscriber sub_pres = node_handler.subscribe("/pressure", 100, &PressureGrabber::GrabPressure, &pgb); 
    // ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);
    
    std::thread sync_thread(&ImageGrabber::SyncWithPressure, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::SyncWithPressure()
{
    while(1)
    {
        if (!img0Buf.empty())
        {
            cv::Mat im;
            double tIm = 0;
            
            tIm = img0Buf.front()->header.stamp.toSec() + mTimeshift_cam_imu;

            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            ros::Time msg_time = img0Buf.front()->header.stamp + ros::Duration(mTimeshift_cam_imu);
            img0Buf.pop();
            this->mBufMutex.unlock();


            mpPGb->mBufMutex.lock();
            // if sensor data available, synchronize readings to image
            if (!mpPGb->pBuf.empty())
            {
                // mpPGb->bufLastVal.fluid_pressure = 0;
                // mpPGb->pressure = 0;
                while(!mpPGb->pBuf.empty() && mpPGb->pBuf.front()->header.stamp.toSec() <= tIm)
                {
                    // TODO: accumulate readings
                    mpPGb->bufLastVal.fluid_pressure = mpPGb->pBuf.front()->fluid_pressure;
                    // TODO: move to within orb_slam
                    mpPGb->pressure = mpPGb->bufLastVal.fluid_pressure;
                    // std::cout << mpPGb->bufLastVal.fluid_pressure << std::endl;
                    mpPGb->pBuf.pop();
                }
            }
            mpPGb->mBufMutex.unlock();

            // do not feed to SLAM system until we get first pressure reading
            if(mpPGb->pressure < 1e-1)
                continue;

            // ORB-SLAM3 runs in TrackMonoUW()
            Sophus::SE3f Tcw = pSLAM->TrackMonoUW(im, tIm, mpPGb->pressure);

            publish_topics(msg_time);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void PressureGrabber::GrabPressure(const sensor_msgs::FluidPressureConstPtr &pressure_msg)
{
    mBufMutex.lock();
    // std::cout << "got pressure message!" << std::endl;
    pBuf.push(pressure_msg);
    mBufMutex.unlock();

    return;
}
