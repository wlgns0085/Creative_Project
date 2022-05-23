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
};
class _compute
{
  private:
    _drone drone;
  public:
    double distance(double P2_latitude, double P2_longitude);
    double bearing(double P2_latitude, double P2_longitude);
};
class _command
{
  private:
    _compute* compute;
    _command* command;
    _drone drone;

    ros::Publisher pub_takeoff, pub_land, pub_control;
    
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

#endif // CONTROLLER_CONTROLLER_H
