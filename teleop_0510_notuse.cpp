#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
//#include <bebop_msgs/Ardrone3PilotingStatemoveToChanged.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>


#define PI 3.141592

geometry_msgs::Twist msg;
std_msgs::Empty empty;
//bebop_msgs::Ardrone3PilotingStatemoveToChanged msg_mtc;

ros::Publisher pub_control;
ros::Publisher pub_takeoff;
ros::Publisher pub_land;
ros::Publisher pub_mtc;

bool state_takeoff = false;

double battery;
double curr_lat = 0.0;
double curr_lon = 0.0;
double curr_alt = 0.0;
double pos_x = 0.0;
double pos_y = 0.0;
double pos_z = 0.0;
double ori_x = 0.0;
double ori_y = 0.0;
double ori_z = 0.0;
double ori_w = 0.0;
double yaw;
double pitch;
double roll;
double att_yaw;
double att_pitch;
double att_roll;
double vel_lin = 0.0;
double vel_ang = 0.0;
double lin_vel = 0.3; // [ m / s ]
double ang_vel = 0.3; // [ radian / s]
double dest_lat;
double dest_lon;
double period;



// ============== Control ==================================================

void takeoff()
{
  std::cout<<"| ### takeoff ! ###"<<std::endl;
  state_takeoff = true;
  pub_takeoff.publish(empty);
}
void land()
{
  std::cout<<"| ### land ! ###"<<std::endl;
  state_takeoff = false;
  pub_land.publish(empty);
}
void control(double lin_x, double lin_y, double lin_z, double ang_z)
{
  msg.linear.x=lin_x;
  msg.linear.y=lin_y;
  msg.linear.z=lin_z;
  msg.angular.x=0.0;
  msg.angular.y=0.0;
  msg.angular.z=ang_z;
  pub_control.publish(msg);
}
void set_dest_gps()
{
  dest_lat = 36.14424294;  // destination latitude
  dest_lon = 128.39389622; // destination longitude
}
void move_length(double length)
{
  period=lin_vel;
  double duration = length / lin_vel;
  ros::Time end_time = ros::Time::now()+ros::Duration(duration);
  ros::Time peri_time = ros::Time::now()+ros::Duration(period);
  while(ros::Time::now()<end_time)
  {
    while(ros::Time::now()>peri_time)
    {
      std::cout<<"time_now    : "<<ros::Time::now()<<std::endl;
      std::cout<<"time_remain : "<<end_time-ros::Time::now()<<std::endl;
      control(lin_vel, 0.0, 0.0, 0.0);
      peri_time = ros::Time::now()+ros::Duration(period);
    }
  }
  control(0.0, 0.0, 0.0, 0.0);
  return;
}
void move_degree(double angle)
{
  period=ang_vel;
  angle=angle*PI/180;
  double duration = angle / ang_vel;
  ros::Time end_time = ros::Time::now()+ros::Duration(duration);
  ros::Time peri_time = ros::Time::now()+ros::Duration(period);

  while(ros::Time::now()<end_time)
  {
    while(ros::Time::now()>peri_time)
    {
      std::cout<<"time_now    : "<<ros::Time::now()<<std::endl;
      std::cout<<"time_remain : "<<end_time-ros::Time::now()<<std::endl;
      control(0.0, 0.0, 0.0, ang_vel);
      peri_time = ros::Time::now()+ros::Duration(period);
    }
  }
  control(0.0, 0.0, 0.0, 0.0);
  return;
}
void bearing_yaw(double target)
{
  // 좌표 변환 (-180~180) to (0~360)
  double _yaw = att_yaw+180;
  double _tar = target+180;
  if (_yaw>=360) _yaw -= _yaw;
  if (_tar>=360) _tar -= _tar;

  int cw;
  if(_tar>_yaw)
  {
    if(_tar-_yaw>=180)
    {
      cw=-1;
    }
    else if (_tar-_yaw<180)
    {
      cw=+1;
      while(_tar>att_yaw+180)
      {
        control(0.0, 0.0, 0.0, cw*ang_vel);
      }
    }
  }
  else if(_tar<_yaw)
  {
    if(_yaw-_tar>=180)
    {
      cw=+1;
    }
    else if(_yaw-_tar<180)
    {
      cw=-1;
      while(_tar<att_yaw+180)
      {
        control(0.0, 0.0, 0.0, cw*ang_vel);
      }
    }
  }
}
void move_to_dest()
{
  //msg_mtc.latitude = float(dest_lat);
  //msg_mtc.longitude = float(dest_lon);
  //msg_mtc.orientation_mode = 1;
  //pub_mtc.publish(msg_mtc);
}
void quaternion_to_euler(double x, double y, double z, double w)
{
  double t0 = 2.0 * (w*x+y*z);
  double t1 = 1.0 - 2.0*(x*x+y*y);
  roll = atan2(t0,t1);
  double t2 = 2.0*(w*y-z*x);
  if(t2>1.0) t2=1.0;
  if(t2<-1.0) t2=-1.0;
  pitch = asin(t2);
  double t3 = 2.0*(w*z+x*y);
  double t4 = 1.0 - 2.0*(y*y+z*z);
  yaw = atan2(t3,t4);
}

// ============== Subscriber ===============================================

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
  ori_w = (msg->pose.pose.orientation.w);

  // velocity
  vel_lin = (msg->twist.twist.linear.x);
  vel_ang = (msg->twist.twist.angular.z);

  quaternion_to_euler(ori_x, ori_y, ori_z, ori_w);
}
void bat_callback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr data)
{
  battery=data->percent;
}
void att_callback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr data)
{
  att_yaw = data->yaw;
  att_pitch = data->pitch;
  att_roll = data->roll;
}

// ============== Print ====================================================

void print_fun()
{
  int result = system("clear");
  
  // GPS status
  std::cout<<"  GPS [ ";
  if (curr_lat==500 && curr_lon==500 && curr_alt==500) std::cout<<"X";
  else std::cout<<"O";
  std::cout<<" ]\t\t\t\t\t\t\t\t";

  // Battery status
  std::cout<<std::right<<battery<<"%  [";
  for (int i = 5; i > (battery + 10) / 20; i--) 
  {
    std::cout <<"□ ";
  }
  for (int i = 1; i < (battery + 10) / 20; i++) 
  {
    std::cout <<"■ ";
  }
  std::cout <<"]"<<std::endl;

  std::cout <<"==========================================================================================="<< std::endl;
  std::cout <<"| gps         -> lat: ["<<curr_lat<<"], lon: ["<<curr_lon<<"], alt: ["<<curr_alt<<"]"<< std::endl;
  std::cout <<"| Destination -> lat: ["<<dest_lat<<"], lon: ["<<dest_lon<<"]"<< std::endl;
  std::cout <<"| "<<std::endl;
  std::cout <<"| Vel         -> Linear: ["<<vel_lin<<"], Angular: ["<<vel_ang<<"]"<< std::endl;
  std::cout <<"| Position    -> x: ["<<pos_x<<"], y: ["<<pos_y<<"], z: ["<<pos_z<<"]"<< std::endl;
  //std::cout <<"| Orientation -> x: ["<<ori_x<<"], y: ["<<ori_y<<"], z: ["<<ori_z<<"]"<<"], w: ["<<ori_w<<"]"<< std::endl;
  //std::cout <<"| Euler -> yaw: ["<<yaw<<"], pitch: ["<<pitch<<"], roll: ["<<roll<<"]"<< std::endl;
  std::cout <<"| "<<std::endl;
  std::cout <<"| Degree      -> yaw: ["<<yaw*180/PI<<"], pitch: ["<<pitch*180/PI<<"], roll: ["<<roll*180/PI<<"]"<< std::endl;
  std::cout <<"| Attitude    -> yaw: ["<<yaw*180/PI<<"], pitch: ["<<pitch*180/PI<<"], roll: ["<<roll*180/PI<<"]"<< std::endl;
  
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
    std::cout<<"|       w                   "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"|   a   s   d               "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| l : land                  "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| t : move length           "<<std::endl;
    std::cout<<"| y : move angle            "<<std::endl;
    std::cout<<"|                           "<<std::endl;
    std::cout<<"| g : move to gps point     "<<std::endl;
    std::cout<<"| p : emergency             "<<std::endl;
    std::cout<<"|                           "<<std::endl;
  }

}

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

// ============== Main =====================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_gps = nh.subscribe("/bebop/fix", 1, gps_callback);
    ros::Subscriber sub_odo = nh.subscribe("/bebop/odom", 1, odo_callback);
    ros::Subscriber sub_bat = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, bat_callback);
    ros::Subscriber sub_att = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, att_callback);

    pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
    pub_control = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,1);

    char key;

    while(ros::ok()){
      ros::spinOnce();

      key = getKey();

      print_fun();

      switch(key)     //Contorl from keyboard input
        {
          case 'o':
          case 'O':
            takeoff();
            break;
          case 'l':
          case 'L':
            land();
            break;
          case 'g':
          case 'G':
            set_dest_gps();
            move_to_dest();
            break;
          case 'w':
          case 'W':
            control(lin_vel, 0.0, 0.0, 0.0);
            break;
          case 's':
          case 'S':
            control(lin_vel*(-1), 0.0, 0.0, 0.0);
            break;
          case 'a':
          case 'A':
            control(0.0, 0.0, 0.0, ang_vel);
            break;
          case 'd':
          case 'D':
            control(0.0, 0.0, 0.0, ang_vel*(-1));
            break;
          case 't':
          case 'T':
            move_length(0.3); // 실험값 : 20~30 cm
            break;
          case 'y':
          case 'Y':
            move_degree(90); // rad(0.3)실험값 : 45도
            break;

        }
    }
    return 0;
}