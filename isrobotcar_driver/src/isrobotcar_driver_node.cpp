#include "ros/ros.h"
#include "cereal_port/CerealPort.h"
#include "string"
#include "iostream"

using namespace cereal;

void streamCallback(std::string * msg)
{
    ROS_INFO("CENAS que recebi |%s|", msg->c_str());
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "isrobotcar_driver_node");

    ROS_INFO("Squirtle for ROS v0.1");
    ros::NodeHandle n;

    CerealPort cena = CerealPort();
    cena.open("/dev/ttyUSB0");
    cena.startReadBetweenStream(streamCallback,':',';');

    ros::Rate r(10);
    ros::Time start_time = ros::Time::now();
    int i = 0;
    while(ros::ok())
    {
        i++;
        if (i== 10)
        {
            char coisas[20];
            int n = sprintf(coisas, ":ahah;");
            ROS_INFO("CENAS que enviei");
            cena.write(coisas,n);
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
