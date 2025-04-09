#pragma once
#include <ros/ros.h>
#include <thread>
#include <fstream>
#include <vector>
#include <deque>


#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include "frontend/ang_vel_estimator.h"
#include "backend/pose_graph_optimizer.h"
// #include "utils/parameters.h"

#include <sensor_msgs/Image.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>


namespace panorama {

class Panorama
{
public:
    Panorama(ros::NodeHandle& nh);
    ~Panorama();

    std::vector<cv::Point3d> precomputed_bearing_vectors; //预计算
    image_geometry::PinholeCameraModel cam;//相机模型
    sensor_msgs::CameraInfo camera_info;



private:
    // Node handle used to subscribe to ROS topics
    ros::NodeHandle nh_;
    // Private node handle for reading parameters
    ros::NodeHandle pnh_;

    ros::Subscriber event_sub_, imu_sub_,camera_info_sub_;
    std::deque<sensor_msgs::Imu> imu_buffer_;  // 缓存最近的IMU数据
    sensor_msgs::Imu imu_data;
    std::mutex imu_mutex_;                     // 线程安全锁



    ros::Time t0_p;
    bool is_t0_p_set_; 

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    bool getClosestImuData(const ros::Time& event_time, sensor_msgs::Imu& imu_out);

    void precomputeBearingVectors();
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);

    bool got_camera_info_;
    
    AngVelEstParams front_end_params_;
    PoseGraphParams back_end_params_;

    AngVelEstimator* ang_vel_estimator_;       // Front-end
    PoseGraphOptimizer* pose_graph_optimizer_; // Back-end


};




}