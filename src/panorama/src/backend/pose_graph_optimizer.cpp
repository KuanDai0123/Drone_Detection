
#include "backend/pose_graph_optimizer.h"
#include "frontend/ang_vel_estimator.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
using namespace std;  

namespace panorama {

PoseGraphOptimizer::PoseGraphOptimizer(ros::NodeHandle* nh): nh_(nh), it_(*nh)
{
    pano_pub_ = it_.advertise("count_pano", 1);
}

PoseGraphOptimizer::~PoseGraphOptimizer()
{
    // delete traj_;
    // delete event_warper_;
    pano_pub_.shutdown();
}

void PoseGraphOptimizer::initialize(int camera_width, int camera_height,
                                    const PoseGraphParams &opt,
                                    std::vector<dvs_msgs::Event>* ptr,
                                    std::vector<cv::Point3d>* precomputed_bearing_vectors_ptr)
{
    // Load params
    params = opt;
    panorama_width_=params.image_opt.panorama_width;
    panorama_height_=params.image_opt.panorama_height;
    count = 0.0;

    count_pano = cv::Mat::zeros(panorama_height_, panorama_width_, CV_32F); 

}

void PoseGraphOptimizer::WarpPano(const ProcessedEventData& data)
{
    Eigen::Vector2d px_mosaic=data.px_mosaic;
    float t_diff = data.t_diff;
    ros::Time timestamp = data.timestamp;

    const int xx = px_mosaic[0],
                yy = px_mosaic[1];
    const float dx = px_mosaic[0] - xx,
                dy = px_mosaic[1] - yy;

    if (1 <= xx && xx < panorama_width_ - 1 && 1 <= yy && yy < panorama_height_ - 1) {
        if (t_diff - count < 1.) {
            count_pano.at<float>(yy, xx) += (1.f - dx) * (1.f - dy);
            count_pano.at<float>(yy, xx + 1) += dx * (1.f - dy);
            count_pano.at<float>(yy + 1, xx) += (1.f - dx) * dy;
            count_pano.at<float>(yy + 1, xx + 1) += dx * dy;
        } else {
            cout<<2<<endl;
            double min_val, max_val;
            // cv::Point min_loc, max_loc;
            // cv::minMaxLoc(count_pano, &min_val, &max_val, &min_loc, &max_loc);
            // cout<<min_val<<' '<<max_val<<endl;
            // 将大于20的值截断为20（类似ReLU的截断效果）
            max_val = 20.0;
            cv::threshold(count_pano, count_pano, max_val, max_val, cv::THRESH_TRUNC);
            show_count_pano(count_pano, max_val, t0_p);
            // show_count_pano(count_pano, max_val, event_buffer[0].ts);
            count_pano = cv::Mat::zeros(panorama_height_, panorama_width_, CV_32F); // 重置为 0
            count += 1.0;
            // t0_p = timestamp.toNSec();
        }
    }

}



void PoseGraphOptimizer::show_count_pano(cv::Mat& count_image, double& max_count, ros::Time timestamp) {
    // 创建灰度图像
    cv::Mat image(panorama_height_, panorama_width_, CV_8UC1);

    // 计算缩放比例，将 count_image 的值映射到 [0, 255]
    int scale = (int)(255 / max_count) + 1;
    count_image.convertTo(image, CV_8UC1, scale); // 将浮点型矩阵转换为 8 位无符号整型矩阵

    // 创建带时间戳的 ROS 消息头
    std_msgs::Header header;
    header.stamp = timestamp;
    // std::cout<<image.type()<<endl;

    // 将 OpenCV 图像转换为 ROS 图像消息
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    // 发布图像消息
    pano_pub_.publish(msg);
}



} // namespace panorama