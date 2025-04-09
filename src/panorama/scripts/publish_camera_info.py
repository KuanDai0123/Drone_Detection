#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    rospy.init_node('dvs_cam_info_pub')
    pub = rospy.Publisher('/dvs/camera_info', CameraInfo, queue_size=1, latch=True)
    
    msg = CameraInfo()
    msg.header.frame_id = "dvs_camera"  # 必须非空，否则某些节点会忽略
    msg.height = 480
    msg.width = 640
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0263186166821032, -0.0123990196445888, 0.0, 0.0, -0.0405981607769830]
    msg.K = [429.733259611991, 0.0, 321.365347200167,
            0.0, 429.841866168175,245.160847403009 ,
            0.0, 0.0, 1.0]
    
    msg.roi.x_offset = 0
    msg.roi.y_offset = 0
    msg.roi.width = msg.width
    msg.roi.height = msg.height

    msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    msg.P = [518.0679, 0, 354.9334, 0, 0, 516.7520, 239.6250, 0, 0, 0, 1, 0]
    
    rospy.loginfo("Publishing CameraInfo to /dvs/camera_info")
    pub.publish(msg)
    rospy.spin()

if __name__ == '__main__':
    publish_camera_info()

