#include <ros/ros.h>
#include <geometry_msgs/Twist.h>		// cmd_vel
#include "rospberrycar_msgs/Status.h"
#include "rospberrycar.h"



void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

ROSpberryCar Ferrari = ROSpberryCar();



int main(int argc, char** argv)
{
    ros::init(argc, argv, "rospberrycar_driver_node");

    ROS_INFO("ROSpberryCar for ROS v0.1");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    string spi_dev_path;
    pn.param<std::string>("spi_dev_path", spi_dev_path, "/dev/spidev0.0");
    double stellaris_version;
    pn.param("stellaris_version", stellaris_version, 1.0);

    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, cmdVelReceived);
    ros::Publisher status_pub = n.advertise<rospberrycar_msgs::Status>("rospberrycar/status", 10);

    // Open SPI channel
    if(Ferrari.initialize(spi_dev_path))
    {
        ROS_FATAL("ROSpberryCar: Device %s not found\n", spi_dev_path.c_str());
        ROS_BREAK();
    }
    ROS_INFO("ROSpberryCar: Initialize complete.");

    // Get Stellaris firmware version
    double mcu_version = Ferrari.getStellarisFirmwareVersion();
    if(mcu_version != stellaris_version)
    {
        ROS_FATAL("ROSpberryCar: Stellaris version %f is incompatible with ROS driver (%f needed).\n", mcu_version, stellaris_version);
        ROS_BREAK();
    }
    ROS_INFO("ROSpberryCar: Correct firmware version (v: %f).", mcu_version);

    ros::Rate r(10);
    ros::Time start_time = ros::Time::now();
    
    while(n.ok())
    {
        //Ferrari.drive(0,0);
        ros::spinOnce();
        r.sleep();
    }

    Ferrari.deInitialize();
    
    return 0;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

}
