#include <cstddef>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bodryi_packetik/CustomImage.h>
#include <bodryi_packetik/camera_switch.h>

const std::string window_name = "view";
bool switch_state = false;
ros::ServiceClient cameraSwitchClient;
//ros::Publisher pub;
int camera_counter = 0;

void imageCallback(const bodryi_packetik::CustomImageConstPtr &msg)
{
  try
  {
    cv_bridge::CvImageConstPtr ros_2_opencv
            = cv_bridge::toCvShare(msg->frame, msg, "bgr8");

    cv::Mat image = ros_2_opencv->image;

    if (!msg->isNewFrame)
    {
        ROS_INFO("Not new frame");
    }

    cv::imshow(window_name, image);
    cv::waitKey(30);

    ROS_INFO("Counter %d", camera_counter);
    if ( camera_counter++ >= 5 )
    {
        camera_counter = 0;

        bodryi_packetik::camera_switch srv_msg;
        srv_msg.request.switcher = switch_state;
        switch_state = !switch_state;

        cameraSwitchClient.call(srv_msg);
//        srv_msg.response.result

    }

//    pub.publish();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'.");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_listener");
  ros::NodeHandle nh;

  cv::namedWindow(window_name);
  cv::startWindowThread();

//  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);
  cameraSwitchClient
          = nh.serviceClient<bodryi_packetik::camera_switch>("camera/switch");
//  pub = nh.advertise();

  ros::spin();
  cv::destroyWindow(window_name);
}
