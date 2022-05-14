#include "ros/ros.h"

#include <cmath>  // for math

#include <sensor_msgs/NavSatFix.h> // for gps
#include <nav_msgs/Odometry.h>     // for odometry

double curr_lat = 0.0;
double curr_lon = 0.0;
double curr_alt = 0.0;

double pos_x = 0.0;
double pos_y = 0.0;
double pos_z = 0.0;

double ori_x = 0.0;
double ori_y = 0.0;
double ori_z = 0.0;

double vel_lin = 0.0;
double vel_ang = 0.0;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr data)
{
  curr_lat = (data->latitude);
  curr_lon = (data->longitude);
  curr_alt = (data->altitude);
}
void odo_callback(const nav_msgs::Odometry::ConstPtr msg)
{
  // position
  pos_x = (msg->pose.pose.position.x);
  pos_y = (msg->pose.pose.position.y);
  pos_z = (msg->pose.pose.position.z);

  // orientation
  ori_x = (msg->pose.pose.orientation.x);
  ori_y = (msg->pose.pose.orientation.y);
  ori_z = (msg->pose.pose.orientation.z);

  // velocity
  vel_lin = (msg->twist.twist.linear.x);
  vel_ang = (msg->twist.twist.angular.z);
}
void print_info()
{

  ROS_INFO("--------------------------------flight information--------------------------------");
  if (curr_lat==500 && curr_lon==500 && curr_alt==500) ROS_INFO("gps        -> lat: [%lf], lon: [%lf], alt: [%lf]    ** GPS not connected **", curr_lat, curr_lon, curr_alt);
  else ROS_INFO("gps        -> lat: [%lf], lon: [%lf], alt: [%lf]", curr_lat, curr_lon, curr_alt);
  ROS_INFO("Position   -> x: [%lf], y: [%lf], z: [%lf]", pos_x, pos_y, pos_z);
  ROS_INFO("Orientation-> x: [%lf], y: [%lf], z: [%lf], w: [%lf]", ori_x, ori_y, ori_z);
  ROS_INFO("Vel        -> Linear: [%lf], Angular: [%lf]", vel_lin, vel_ang);
}
void set_dest_gps()
{
  const double dest_lat = 36.14424294;  // destination latitude
  const double dest_lon = 128.39389622; // destination longitude
}
void move_to_dest()
{
  // angular
  //double angular = 0;

  // linear
  //double length = sqrt( pow(dest_lat - curr_lat,2) + pow(dest_lon - curr_lon,2) );


}

// todo
// move to destination
//  - 현재 각도 구하기
//  - 목표 각도와 현재 각도 차이만큼 angular 회전
//  - 목표 지점과의 거리 계산 후 linear 직진
//
// twist - vector3 : linear(m/s), angular(rad/s)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("/bebop/fix", 100, gps_callback);
    ros::Subscriber sub_odo = nh.subscribe("/bebop/odom", 100, odo_callback);

    
    while(ros::ok()){
      print_info();

      ros::spinOnce();
      ros::Rate(1).sleep(); 

      set_dest_gps();
      move_to_dest();
    }


    return 0;
}
