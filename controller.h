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
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>
#include <bebop_msgs/Ardrone3GPSStateNumberOfSatelliteChanged.h>
#define PI 3.141592

ros::Publisher pub_takeoff, pub_land, pub_control;
ros::Subscriber sub_pos, sub_att, sub_bat, sub_sat;

geometry_msgs::Twist twist;
std_msgs::Empty empty;

double distance_threshold = 3.0; // meter
double bearing_threshold = 15.0; // degree

bool print_option = true;

// ============== getKey ===================================================
char getKey()     //get keyboard input
{                                           //this code can get keyboard input one charactor
                                            //So you can't use some command like ctrl+c
    struct termios oldt, newt;  
    int ch;
    std_msgs::String msg;
    tcgetattr(STDIN_FILENO, &oldt); //get termianl information | tcgetattr(int filds, struct termios *termios_p)
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);   //set flag to non canonical mode and no print input char
                                        //canonical mode requires the user to press the Enter key to enter a character
                                        //non canonical mode does not require Enter key input.
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // TCSANOW : Immediately reflects the attributes
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return (char)ch;
}
// ============== getKey ===================================================
struct _destination {
  double latitude, longitude, altitude;
};
struct _drone
{
  double latitude, longitude, altitude;
  double yaw, pitch, roll;
  double linearspeed, angularspeed;

  struct _destination destination;
  
  int state;
  // 1: land
  // 2: hovering
  // 3: moving

  int command;
  // 1: takeoff
  // 2: land
  // 3: move to point ( lon, lat )
  // 4: flight plan ( route[] )

  double battery;
  unsigned int satellite;
}drone;
class _compute
{
  private:
    
  public:
    double distance(double P2_latitude, double P2_longitude);
    double bearing(double P2_latitude, double P2_longitude);
};
class _command
{
  private:
    _compute* compute;
    _command* command;
  public:
    
    void takeoff();
    void land();
    void control();
    void control_lin(double linear);
    void control_ang(double angular);
    void stop();
    void bearing(double target);
    void goTo(double latitude, double longitude);
    void flightplan(double route);
};
class _print
{
  private:

  public:
    void string(const char* string);
    void command(const char* command_string);
};


void _command::takeoff()
{
  pub_takeoff.publish(empty);
}
void _command::land()
{
  pub_land.publish(empty);
}
void _command::control()
{
  twist.linear.x=drone.linearspeed;
  twist.angular.z=drone.angularspeed;
  pub_control.publish(twist);
}
void _command::control_lin(double linear)
{
  twist.linear.x=linear;
  twist.angular.z=0.0;
  pub_control.publish(twist);
}
void _command::control_ang(double angular)
{
  twist.linear.x=0.0;
  twist.angular.z=angular;
  pub_control.publish(twist);
}
void _command::stop()
{
  twist.linear.x=0.0;
  twist.angular.z=0.0;
  pub_control.publish(twist);
}
void _command::bearing(double target)
{
  int ccw;

  if(target>drone.yaw)
  {
    if(target-drone.yaw>=180)
    {
      ccw=+1;
      while(drone.yaw>target || drone.yaw<target-90)
      {
        control_ang(ccw*0.3);
        ros::spinOnce();
      }
    }
    else if(target-drone.yaw<180)
    {
      ccw=-1;
      while(target>drone.yaw)
      {
        control_ang(ccw*0.3);
        ros::spinOnce();
      }
    }
  }
  if(target<drone.yaw)
  {
    if(drone.yaw-target>=180)
    {
      ccw=-1;
      while(drone.yaw<target || drone.yaw>target+90)
      {
        control_ang(ccw*0.3);
        ros::spinOnce();
      }
    }
    else if(drone.yaw-target<180)
    {
      ccw=+1;
      while(target<drone.yaw)
      {
        control_ang(ccw*0.3);
        ros::spinOnce();
      }
    }
  }
}
void _command::goTo(double latitude, double longitude)
{
  ros::spinOnce();
  double distance = compute->distance(latitude, longitude);
  double bearingangle = compute->bearing(latitude, longitude);

  while(distance>distance_threshold)
  {
    ros::spinOnce();
    distance = compute->distance(latitude, longitude);
    bearingangle = compute->bearing(latitude, longitude);

    if(fabs(bearingangle-drone.yaw)>bearing_threshold)
    {
      bearing(bearingangle);
    }
    control_lin(0.3);
  }
}
void _command::flightplan(double route)
{
  goTo(drone.latitude, drone.longitude);
}
double _compute::distance(double P2_latitude, double P2_longitude)
{
  ros::spinOnce();
  double P1_latitude = drone.latitude;
  double P1_longitude = drone.longitude;
  if ((P1_latitude == P2_latitude) && (P1_longitude == P2_longitude)) return 0;
  double e10 = P1_latitude * PI / 180;
  double e11 = P1_longitude * PI / 180;
  double e12 = P2_latitude * PI / 180;
  double e13 = P2_longitude * PI / 180;
  double c16 = 6356752.314140910;
  double c15 = 6378137.000000000;
  double c17 = 0.0033528107;
  double f15 = c17 + c17 * c17, f16 = f15 / 2;
  double f17 = c17 * c17 / 2, f18 = c17 * c17 / 8, f19 = c17 * c17 / 16;
  double c18 = e13 - e11, c20 = (1 - c17) * tan(e10);
  double c21 = atan(c20), c22 = sin(c21),c23 = cos(c21);
  double c24 = (1 - c17) * tan(e12);
  double c25 = atan(c24), c26 = sin(c25), c27 = cos(c25), c29 = c18;
  double c31 = (c27 * sin(c29) * c27 * sin(c29))
      + (c23 * c26 - c22 * c27 * cos(c29)) * (c23 * c26 - c22 * c27 * cos(c29));
  double c33 = (c22 * c26) + (c23 * c27 * cos(c29));
  double c35 = sqrt(c31) / c33, c36 = atan(c35), c38 = 0, c40 = 0;
  if (c31 == 0) c38 = 0;
  else c38 = c23 * c27 * sin(c29) / sqrt(c31);
  if ((cos(asin(c38)) * cos(asin(c38))) == 0) c40 = 0;
  else c40 = c33 - 2 * c22 * c26 / (cos(asin(c38)) * cos(asin(c38)));
  double c41 = cos(asin(c38)) * cos(asin(c38)) * (c15 * c15 - c16 * c16) / (c16 * c16);
  double c43 = 1 + c41 / 16384
      * (4096 + c41 * (-768 + c41 * (320 - 175 * c41)));
  double c45 = c41 / 1024 * (256 + c41 * (-128 + c41 * (74 - 47 * c41)));
  double c47 = c45 * sqrt(c31) * (c40 + c45 / 4
      * (c33 * (-1 + 2 * c40 * c40) - c45 / 6 * c40
      * (-3 + 4 * c31) * (-3 + 4 * c40 * c40)));
  double c50 = c17 / 16 * cos(asin(c38)) * cos(asin(c38))
      * (4 + c17 * (4 - 3 * cos(asin(c38)) * cos(asin(c38))));
  double c52 = c18 + (1 - c50) * c17 * c38
      * (acos(c33) + c50 * sin(acos(c33)) * (c40 + c50 * c33 * (-1 + 2 * c40 * c40)));
  double c54 = c16 * c43 * (atan(c35) - c47);
  return c54; // return distance in meter
}
double _compute::bearing(double P2_latitude, double P2_longitude)
{
  ros::spinOnce();
  double P1_latitude = drone.latitude;
  double P1_longitude = drone.longitude;
  double Cur_Lat_radian = P1_latitude * (3.141592 / 180);
  double Cur_Lon_radian = P1_longitude * (3.141592 / 180);
  double Dest_Lat_radian = P2_latitude * (3.141592 / 180);
  double Dest_Lon_radian = P2_longitude * (3.141592 / 180);
  double radian_distance = 0;
  radian_distance = acos(sin(Cur_Lat_radian)
      * sin(Dest_Lat_radian) + cos(Cur_Lat_radian)
      * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian));
  double radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian)
      * cos(radian_distance)) / (cos(Cur_Lat_radian) * sin(radian_distance)));
  double true_bearing = 0;
  if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0){
      true_bearing = radian_bearing * (180 / 3.141592);
      true_bearing = 360 - true_bearing;
  }
  else{
      true_bearing = radian_bearing * (180 / 3.141592);
  }
  return true_bearing;
}
void _print::string(const char* string)
{
  if(print_option==true)
  {
    std::cout<<string<<std::endl;
  }
}
void _print::command(const char* command_string)
{
  if(print_option==true)
  {
    std::cout<<"Command : "<<command_string<<std::endl;
  }
}

#endif // CONTROLLER_CONTROLLER_H
