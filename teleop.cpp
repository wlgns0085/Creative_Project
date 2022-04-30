#include "ros/ros.h"			// ROS 기본 헤더 파일
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr data)
{
  float gps_lat = data->latitude;
  float gps_lon = data->longitude;
  float gps_alt = data->altitude;

  ROS_INFO("latitude  = %lf", gps_lat);
  ROS_INFO("longitude = %lf", gps_lon);
  ROS_INFO("altitude  = %lf", gps_alt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/bebop/fix", 100, gps_callback);

    ros::spin();

    return 0;
}