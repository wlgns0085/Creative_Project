#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>
#include <bebop_msgs/Ardrone3GPSStateNumberOfSatelliteChanged.h>

#define PI 3.141592

struct _drone
{
  double latitude, longitude, altitude;
  double yaw, pitch, roll;
  double linearspeed, angularspeed;
  
  int state;
  // X1: land
  // X2: stop (hovering)
  // X3: move (flying)

  // 0X: safe
  // 9X: emergency (stop)

  int command;
  // 1: takeoff
  // 2: land
  // 3: move to point ( lon, lat )
  // 4: flight plan ( route[] )
  // 99: emergency ( stop )
  // 9: confirm safety

  // 81: w, 82: s, 83: a, 84: d

  int battery;
  unsigned int satellite;
}drone;

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

ros::Publisher pub_command;
std_msgs::Int16 msg;
int prev_com = 0;

void publish_command(int num)
{
  prev_com = num;
  msg.data = num; 
  pub_command.publish(msg);
}
void print_state();
void call_battery(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr data) // 베터리 잔량
{
  drone.battery=data->percent;
}
void call_satellite(const bebop_msgs::Ardrone3GPSStateNumberOfSatelliteChanged::ConstPtr data) // 위성 숫자
{
  drone.satellite = data->numberOfSatellite;
}
void call_gps(const bebop_msgs::Ardrone3PilotingStatePositionChanged::ConstPtr data) // GPS 좌표
{
  drone.latitude = data->latitude;
  drone.longitude = data->longitude;
  drone.altitude = data->altitude;
}
void call_attitude(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr data) // yaw, pitch, roll
{
  drone.yaw = (data->yaw)*180/PI;
  if (drone.yaw<0) drone.yaw+=360;
  drone.pitch = (data->pitch)*180/PI;
  drone.roll = (data->roll)*180/PI;

  print_state();
}
void print_state()
{
  int result = system("clear");

  std::cout <<"┌─────────────────────────────────────────────────────────────────────────────────┐"<<std::endl;
  std::cout<<"│ Flight [";
  if(drone.state==1) std::cout<<" land ___ ]";
  else std::cout<<" fly --- ]";
      
  // GPS status
  std::cout<<"  GPS [ ";
  if (drone.longitude==500 && drone.latitude==500 && drone.altitude==500) std::cout<<"X";
  else std::cout<<"O";
  std::cout<<" ]  Sat [ "<<drone.satellite;
  std::cout<<" ]\t\t      ";


  // Battery status
  printf("  %3d%% [",drone.battery);
  for (int i = 5; i >= (drone.battery + 10) / 20; i--) 
  {
    std::cout <<"□ ";
  }
  for (int i = 1; i < (drone.battery + 10) / 20; i++) 
  {
    std::cout <<"■ ";
  }
  std::cout <<"] │"<<std::endl;
  
  // Values
  std::cout<<"├─────────────────────────────────────────────────────────────────────────────────┤"<<std::endl;
  printf("│ GPS          -> lon: [%12.8lf]  lat: [%12.8lf]  alt: [%11.8lf]   │\n",drone.longitude, drone.latitude, drone.altitude);
  printf("│ Attitude     -> yaw: [%7.3lf]     pitch: [%8.3lf]     roll: [%8.3lf]       │\n",drone.yaw,drone.pitch,drone.roll);
  
  
  std::cout<<"│ Previous Command : ";
  switch(prev_com)
  {
    case 1: std::cout<<"takeoff                                                      │"<<std::endl; break;
    case 2: std::cout<<"land                                                         │"<<std::endl; break;
    case 3: std::cout<<"move to point                                                │"<<std::endl; break;
    case 4: std::cout<<"flight plan                                                  │"<<std::endl; break;
    case 99: std::cout<<"emergency (stop)                                             │"<<std::endl; break;
    case 9: std::cout<<"confirm safety                                               │"<<std::endl; break;
    default: std::cout<<"                                                             │"<<std::endl; break;
  }
  
  std::cout<<"├─────────────────────────────────────────────────────────────────────────────────┤"<<std::endl;
  std::cout<<"│                                                                                 │"<<std::endl;
  std::cout<<"│        [w]            [t] move to point       [o] takeoff  [p] emergency (stop) │"<<std::endl;
  std::cout<<"│     [a][s][d]         [g] flight plan         [l] land                          │"<<std::endl;
  std::cout<<"│                                                            [m] confirm safety   │"<<std::endl;
  std::cout<<"│                                                                                 │"<<std::endl;
  std::cout<<"└─────────────────────────────────────────────────────────────────────────────────┘"<<std::endl;

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle nh;

  ros::Subscriber sub_pos = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 1, call_gps);
  ros::Subscriber sub_att = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, call_attitude);
  ros::Subscriber sub_bat = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, call_battery);
  ros::Subscriber sub_sat = nh.subscribe("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 1, call_satellite);

  pub_command = nh.advertise<std_msgs::Int16>("keyboard/command", 1);

  drone.state=1;
  
  //ros::spin();
  char key;
  while(ros::ok()){
    ros::spinOnce();
    
    key = getKey();
    switch(key)
    {
      case 'w':
      case 'W':
        publish_command(81);
        break;
      case 's':
      case 'S':
        publish_command(82);
        break;
      case 'a':
      case 'A':
        publish_command(83);
        break;
      case 'd':
      case 'D':
        publish_command(84);
        break;
      case 'o': // takeoff
      case 'O':
        publish_command(1);
        drone.state=2;
        break;
      case 'l': // land
      case 'L':
        publish_command(2);
        drone.state=1;
        break;
      case 't': // move to point ( lon, lat )
      case 'T':
        publish_command(3);
        break;
      case 'g': // flight plan ( route[] )
      case 'G':
        publish_command(4);
        break;
      case 'p': // emergency (stop)
      case 'P':
        publish_command(99);
        break;
      case 'm': // confirm safety
      case 'M':
        publish_command(9);
        break;
      default:
        ros::spinOnce();
    }
  }
  std::cout<<("Controller Exited")<<std::endl;
  ros::waitForShutdown();
  return 0;
}