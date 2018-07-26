#include "ros/ros.h"
#include "haha_package/TMsg.h"

void chat_handler (const haha_package::TMsg::ConstPtr &msg)
{
    ROS_INFO( "Tick: [%d]", msg->num );
    ROS_INFO( "Str: [%s]", msg->str.c_str() );

//    ROS_INFO( "Tick: [%d]", msg->counter );
//    if ( msg->err_exist )
//    {
//        ROS_INFO( "\tError: [%s]", msg->err_msg.c_str() );
//        ROS_INFO( "\tTime: [%d]", msg->err_moment.sec );
//    }
}

int main( int argc, char **argv )
{
	ros::init( argc, argv, "linker" );

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("chat", 100, chat_handler);

	ros::spin();

	// ros::spinOnce();

	return 0;
}
