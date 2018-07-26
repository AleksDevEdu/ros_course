#include "ros/ros.h"
#include "haha_package/TMsg.h"

//#include "std_msgs/String.h"

#include <sstream>

#include "vectorsample.h"

int main( int argc, char **argv )
{
    VectorSample vector_sample (10);

//    vector_sample.test_front();
//    vector_sample.test_back();
//    vector_sample.test_at(5);
//    vector_sample.test_pop_back();
    vector_sample.test_op_equal();

    VectorSample::show_message();

    return 0;






	ros::init( argc, argv, "tinker" );
 
	ros::NodeHandle node;

    ros::Publisher tink_pub = node.advertise<haha_package::TMsg>("chat", 100);

    ros::Rate rate(1); //Hz

	int count = 0;

    haha_package::TMsg msg;

	while(ros::ok())
	{
        if ( count % 6 == 0 )
        {
            std::stringstream sst;
            sst << "Achtung! Something happened!! =(";
            msg.str = sst.str();
//            msg.err_moment = ros::Time::now();
        } else {
            msg.str = "Ok";
        }

        msg.num = count;

		tink_pub.publish( msg );

		rate.sleep();
		count++;
	}

	return 0;
}
