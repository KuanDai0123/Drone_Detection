#pragma once
#include "frontend/ang_vel_estimator.h"
#include "utils/parameters.h"

#include <opencv2/core.hpp>
#include <Eigen/Core>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <thread>
#include <atomic>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace panorama {

class AngVelEstimator;

class PoseGraphOptimizer {
public:
    // Constructor
    PoseGraphOptimizer(ros::NodeHandle* nh);

    // Deconstructor
    ~PoseGraphOptimizer();

    // Initialize the backend
    void initialize(int camera_width, int camera_height,
                    const PoseGraphParams& val,
                    std::vector<dvs_msgs::Event>* ptr,
                    std::vector<cv::Point3d>* precomputed_bearing_vectors_ptr);
    void setInitialTimestamp(const ros::Time& t0) { t0_p = t0; }

    // Set the pointer to the frontend
    void setFrontend(AngVelEstimator* ptr) { ang_vel_estimator_ = ptr; }

    void WarpPano(const ProcessedEventData& data);
    void show_count_pano(cv::Mat& count_image, double& max_count, ros::Time timestamp) ;

    PoseGraphParams params;
    std::mutex mutex_events, mutex_ang_vel;



private:

    ros::NodeHandle* nh_;

    image_transport::ImageTransport it_;
    image_transport::Publisher pano_pub_;

    typedef long long int sll;
    ros::Time t0_p;
    sll t0_p_;

    cv::Mat count_pano;
    float count;

    AngVelEstimator* ang_vel_estimator_;
    int panorama_width_;
    int panorama_height_;



};

} // namespace panorama