#include <ros/ros.h>
#include <bodryi_packetik/Bodryi.h>
#include <iostream> // cout

//using namespace ros;

const static int LENA_ID    = 1;
const static int KOSTYA_ID  = 2;

int main ( int argc, char **argv )
{
    ros::init( argc, argv, "net_sender" );

    ros::NodeHandle node;

    ros::Publisher pub =
        node.advertise<bodryi_packetik::Bodryi>
            ("secrets", 100);

    ros::Duration               dur(2, 0);
    ros::Rate                   rate(dur);
    bodryi_packetik::Bodryi     msg;

    bool person_switch = false;

    while(1)
    {   // &&  ||  !
        // & | ~
        msg.id = person_switch /*bool condition*/ ?
                 LENA_ID /*if true*/ :
                 KOSTYA_ID /*if false*/;

        if ( person_switch )
        {
            msg.msg = "Hi, Lena!";
        } else {
            msg.msg = "Zdorova, bodryi!";
        }

        person_switch = !person_switch;

        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}

