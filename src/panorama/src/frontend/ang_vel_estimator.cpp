#include <ros/ros.h>

#include "backend/pose_graph_optimizer.h"
#include "frontend/ang_vel_estimator.h"
#include "utils/parameters.h"

#include <camera_info_manager/camera_info_manager.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <glog/logging.h>
#include <Eigen/Dense> 

#include <iostream>
using namespace std;  


namespace panorama {

AngVelEstimator::AngVelEstimator(ros::NodeHandle* nh): nh_(nh), it_(*nh)
{
    // Set publishers
    // img_pub_ = it_.advertise("local_iwe", 1);
    // ang_vel_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("/dvs/angular_velocity", 1);

    // // Initial value of motion parameters velocity
    // ang_vel_ = cv::Point3d(0.,0.,0.);
}

AngVelEstimator::~AngVelEstimator()
{
    // img_pub_.shutdown();
    // ang_vel_pub_.shutdown();
}


void AngVelEstimator::initialize(image_geometry::PinholeCameraModel* cam,
                                 const AngVelEstParams& val,
                                 const std::vector<cv::Point3d>& precomputed_bearing_vectors)
{
    // Load camera information
    cam_width_ = cam->fullResolution().width;
    cam_height_ = cam->fullResolution().height;
    camera_matrix_ = cam->fullIntrinsicMatrix();

    // Load params
    params = val;
    // Get the pre-computed bearing vector
    precomputed_bearing_vectors_ = precomputed_bearing_vectors;

    R_matrix = params.image_opt.R_matrix;

    center_ = Eigen::Vector2d(
        (double)params.image_opt.panorama_width / 2.0, 
        (double)params.image_opt.panorama_height
    );

    if (R_matrix.size() == 9) {
        Eigen::Map<const Eigen::Matrix3d> R_raw(R_matrix.data());
        R = R_raw.transpose();  // ROS数组是行优先，需转置为列优先
    } else {
        ROS_ERROR("Invalid rotation matrix size");
        R = Eigen::Matrix3d::Identity();
    }

    // Initialize the event maintainance (containers and lists)
    std::unique_lock<std::mutex> ev_lock(pose_graph_optimizer_->mutex_events);
    events_.clear();

    ev_lock.unlock();

}

void AngVelEstimator::updateRotationMatrix(const sensor_msgs::Imu& imu) {
    if (is_first_imu_) {
        last_imu_time_ = imu.header.stamp;
        is_first_imu_ = false;
        return;
    }

    double dt = (imu.header.stamp - last_imu_time_).toSec();
    last_imu_time_ = imu.header.stamp;

    // 角速度减去零漂
    Eigen::Vector3d omega(
        imu.angular_velocity.x,
        imu.angular_velocity.y,
        imu.angular_velocity.z
    );

    // 计算旋转增量
    Eigen::Vector3d theta = omega * dt;
    if (theta.norm() > 1e-6) {  // 避免零旋转
        Eigen::Matrix3d delta_R = Eigen::AngleAxisd(theta.norm(), theta.normalized()).toRotationMatrix();
        R_t = R_t * delta_R;
        R_t = R_t.householderQr().householderQ();  // 正交化
    }
}


ProcessedEventData AngVelEstimator::processEvent(const dvs_msgs::Event& ev)
{
    const cv::Point3d bvec = precomputed_bearing_vectors_.at(ev.y*cam_width_ + ev.x);
    Eigen::Vector3d e_ray_cam(bvec.x, bvec.y, bvec.z);

    ccc+=1;
    
    if (ccc % 100000 == 0){
        cout<<R_t<<endl;
    } 

    float t_diff = double(ev.ts.toNSec() - t0_p.toNSec()) / 1000000000.0;

    double cos_term = cos(2 * pi * t_diff);
    double sin_term = sin(2 * pi * t_diff);
    Eigen::Matrix3d R_eigen_;

    R_eigen_ << cos_term, R(2, 1) * sin_term, R(2, 2) * sin_term,
                0, R(1, 1), R(1, 2),
                -sin_term, R(2, 1) * cos_term, R(2, 2) * cos_term;

    Eigen::Vector3d e_ray_w = R_eigen_ * e_ray_cam;
    Eigen::Vector2d px_mosaic;

    // Calculate the pixel position in the panorama image
    const double phi = std::atan2(e_ray_w[0], e_ray_w[2]);
    const double theta = std::asin(e_ray_w[1] / e_ray_w.norm());


    px_mosaic = center_ + Eigen::Vector2d(-phi * params.image_opt.panorama_width / (2.0 * pi), -theta * params.image_opt.panorama_height / (pi*57.99/180));

    return ProcessedEventData{
        .timestamp = ev.ts,
        .px_mosaic = px_mosaic,
        .t_diff = t_diff
    };

}


} // namespace panorama