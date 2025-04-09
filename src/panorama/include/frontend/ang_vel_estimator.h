#pragma once

#include "backend/pose_graph_optimizer.h"
#include "utils/parameters.h"

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>



namespace panorama {

class PoseGraphOptimizer;

class AngVelEstimator {
public:
    // Constructor
    AngVelEstimator(ros::NodeHandle* nh);

    // Deconstructor
    ~AngVelEstimator();
    

    void initialize(image_geometry::PinholeCameraModel* cam,
                                 const AngVelEstParams& val,
                                 const std::vector<cv::Point3d>& precomputed_bearing_vectors);
    void setInitialTimestamp(const ros::Time& t0) { t0_p = t0; }
    
    void setBackend(PoseGraphOptimizer* ptr) { pose_graph_optimizer_ = ptr; }
    ProcessedEventData processEvent(const dvs_msgs::Event& ev);
    void updateRotationMatrix(const sensor_msgs::Imu& imu);
    AngVelEstParams params;
    std::vector<dvs_msgs::Event> events_;
    

private:
    // Pointer to the ROS node, for publishing
    ros::NodeHandle* nh_;
    image_transport::ImageTransport it_;

    Eigen::Matrix3d R_t = Eigen::Matrix3d::Identity();  // 初始化为单位矩阵
    ros::Time last_imu_time_;
    bool is_first_imu_ = true;

    typedef long long int sll;
    double pi = M_PI;
    ros::Time t0_p;
    sll t0_p_;

    int cam_width_, cam_height_;
    Eigen::Matrix3d R;
    std::vector<double> R_matrix;
    cv::Matx33d camera_matrix_;
    Eigen::Vector2d center_;

    std::vector<cv::Point3d> precomputed_bearing_vectors_;

    PoseGraphOptimizer* pose_graph_optimizer_;


    int ccc=0;


};

} // namespace panorama