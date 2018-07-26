#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <bodryi_packetik/CustomImage.h>
#include <bodryi_packetik/camera_switch.h>

bool isFrameNew = true;

bool _switch(bodryi_packetik::camera_switch::Request &req,
             bodryi_packetik::camera_switch::Response &resp)
{
    ROS_INFO("Got switch service request");
    isFrameNew = req.switcher;
    resp.result = true;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
//    image_transport::ImageTransport it(nh);
    ros::Publisher pub
            = nh.advertise<bodryi_packetik::CustomImage>("camera/image", 1);

    ros::ServiceServer service = nh.advertiseService("camera/switch", _switch);

    cv::VideoCapture cap(2);
    if(!cap.isOpened())
    {
        ROS_INFO("Can`t open camera!");
        return 1;
    }

    cv::Mat frame;
    bodryi_packetik::CustomImage msg;
    sensor_msgs::ImagePtr pImageMsg;
    ros::Rate loop_rate(1);

    while (nh.ok()) {
        if (isFrameNew)
            cap >> frame;

        if(!frame.empty()) {
            cv_bridge::CvImage opencv_2_ros (std_msgs::Header(), "bgr8", frame);
            pImageMsg = opencv_2_ros.toImageMsg();

            msg.frame       = *pImageMsg;
            msg.isNewFrame  = isFrameNew;

            pub.publish(msg);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
