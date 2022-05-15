#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>
#include <bebop_msgs/Ardrone3GPSStateNumberOfSatelliteChanged.h>


#define PI 3.141592

geometry_msgs::Twist msg;
std_msgs::Empty empty;

ros::Publisher pub_control;
ros::Publisher pub_takeoff;
ros::Publisher pub_land;
ros::Publisher pub_mtc;

bool state_takeoff = false;
double battery;
uint8_t sat_num;
double lin_vel_curr, ang_vel_curr;
double lat_curr, lon_curr, alt_curr;
double lat_chng, lon_chng, alt_chng;
double lat_dest, lon_dest, alt_dest;
double yaw_curr, pitch_curr, roll_curr;
double yaw_chng, pitch_chng, roll_chng;
double pos_x, pos_y, pos_z;
double ori_x, ori_y, ori_z, ori_w;
double spd_x, spd_y, spd_z;
double dest_dist, dest_bear;

// parameter

double period;
double lin_vel = 0.3; // [ m / s ]
double ang_vel = 0.3; // [ radian / s]

// ============== Print ====================================================

void print_simple_stat()
{
  //int result = system("clear");
  
  // Flight status
  std::cout<<"  Flight [";
  if(state_takeoff==true) std::cout<<" fly --- ]";
  else if(state_takeoff==false) std::cout<<" land ___ ]";

  // GPS status
  std::cout<<"  GPS [ ";
  if (lat_curr==500 && lon_curr==500 && alt_curr==500) std::cout<<"X";
  else std::cout<<"O";
  std::cout<<" ]  Sat [ "<<sat_num;
  std::cout<<" ]\t\t\t";

  // Battery status
  std::cout<<std::right<<battery<<"%  [";
  for (int i = 5; i >= (battery + 10) / 20; i--) 
  {
    std::cout <<"□ ";
  }
  for (int i = 1; i < (battery + 10) / 20; i++) 
  {
    std::cout <<"■ ";
  }
  std::cout <<"]"<<std::endl;

  std::cout <<"==========================================================================================="<< std::endl;
}
void print_stat()
{
  std::cout <<"| GPS          -> lat: ["<<lat_curr<<"], lon: ["<<lon_curr<<"], alt: ["<<alt_curr<<"]"<< std::endl;
  std::cout <<"| Changed      -> lat: ["<<lat_chng<<"], lon: ["<<lon_chng<<"], alt: ["<<alt_chng<<"]"<< std::endl;
  //std::cout <<"| Destination -> lat: ["<<lat_dest<<"], lon: ["<<lon_dest<<"]"<< std::endl;
  std::cout <<"| "<<std::endl;
  //std::cout <<"| Position    -> x: ["<<pos_x<<"], y: ["<<pos_y<<"], z: ["<<pos_z<<"]"<< std::endl;
  //std::cout <<"| Orientation -> x: ["<<ori_x<<"], y: ["<<ori_y<<"], z: ["<<ori_z<<"]"<<"], w: ["<<ori_w<<"]"<< std::endl;
  //std::cout <<"| Euler -> yaw: ["<<yaw<<"], pitch: ["<<pitch<<"], roll: ["<<roll<<"]"<< std::endl;
  std::cout <<"| Degree       -> yaw: ["<<yaw_curr*180/PI<<"], pitch: ["<<pitch_curr*180/PI<<"], roll: ["<<roll_curr*180/PI<<"]"<< std::endl;
  std::cout <<"| Attitude     -> yaw: ["<<yaw_chng*180/PI<<"], pitch: ["<<pitch_chng*180/PI<<"], roll: ["<<roll_chng*180/PI<<"]"<< std::endl;
  std::cout <<"| "<<std::endl;
  std::cout <<"| Velocity     -> linear: ["<<lin_vel_curr<<"], angular: ["<<ang_vel_curr<<"]"<< std::endl;
  std::cout <<"| Speed on map -> x: ["<<spd_x<<"], y: ["<<spd_y<<"], z: ["<<spd_z<<"]"<< std::endl;
  std::cout <<"==========================================================================================="<< std::endl;
  
  if (state_takeoff==false)
  {
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| o : takeoff               "<<std::endl;
    std::cout<<"|                           "<<std::endl;
  }
  if (state_takeoff==true)
  {
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| l : land                  "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| w, s : forward, backward  "<<std::endl;
    std::cout<<"| a, d : rotate             "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| t : move length           "<<std::endl;
    std::cout<<"| y : move angle            "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| g : set gps & go          "<<std::endl;
    std::cout<<"|                           "<<std::endl;
  }

}

char getKey();

#endif