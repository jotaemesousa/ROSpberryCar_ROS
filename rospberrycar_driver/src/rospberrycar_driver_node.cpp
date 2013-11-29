 

 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>		// cmd_vel
#include "rospberrycar_msgs/Status.h"
#include "rospberrycar.h"

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rospberrycar_driver_node");

    ROS_INFO("ROSpberryCar for ROS v0.1");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, cmdVelReceived);
    ros::Publisher status_pub = n.advertise<rospberrycar_msgs::Status>("rospberrycar/status", 10);

    
    ros::Rate r(10);
    ros::Time start_time = ros::Time::now();
    
    while(n.ok())
    {
      
	ros::spinOnce();
	r.sleep();
    }
    
    return 0;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  
}