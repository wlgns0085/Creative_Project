#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#define PI 3.141592

geometry_msgs::Twist msg;
std_msgs::Empty empty;

ros::Publisher pub_control;
ros::Publisher pub_takeoff;
ros::Publisher pub_land;

bool state_takeoff = false;

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
double lin_vel = 0.3; // [ m / s ]
double ang_vel = 0.3; // [ radian / s]

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
  const double dest_lat = 36.14424294;  // destination latitude
  const double dest_lon = 128.39389622; // destination longitude
}
void move_length(double length)
{
  double duration = length / lin_vel;
  ros::Time end_time = ros::Time::now()+ros::Duration(duration);

  while(ros::Time::now()<end_time)
  {
    control(lin_vel, 0.0, 0.0, 0.0);
  }

}
void move_angle(double angle)
{
  if (angle<0) angle = abs(angle);
  else if (angle>PI) angle -= PI;

  double duration = angle / ang_vel;
  ros::Time end_time = ros::Time::now()+ros::Duration(duration);

  while(ros::Time::now()<end_time)
  {
    control(0.0, 0.0, 0.0, ang_vel);
  }
}
void move_to_dest()
{

}

// ============== Subscriber & Publisher ===================================

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

// ============== Print ====================================================

void print_fun()
{
  int result = system("clear");
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
  
  std::cout <<"--------------------------------flight information--------------------------------"<< std::endl;
  if (curr_lat==500 && curr_lon==500 && curr_alt==500)
  std::cout <<"| gps         -> lat: ["<<curr_lat<<"], lon: ["<<curr_lon<<"], alt: ["<<curr_alt<<"]    ** GPS not connected **"<< std::endl;
  else
  std::cout <<"| gps         -> lat: ["<<curr_lat<<"], lon: ["<<curr_lon<<"], alt: ["<<curr_alt<<"]"<< std::endl;
  std::cout <<"| Position    -> x: ["<<pos_x<<"], y: ["<<pos_y<<"], z: ["<<pos_z<<"]"<< std::endl;
  std::cout <<"| Orientation -> x: ["<<ori_x<<"], y: ["<<ori_y<<"], z: ["<<ori_z<<"]"<< std::endl;
  std::cout <<"| Vel         -> Linear: ["<<vel_lin<<"], Angular: ["<<vel_ang<<"]"<< std::endl;
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
    ros::Subscriber sub_gps = nh.subscribe("/bebop/fix", 10, gps_callback);
    ros::Subscriber sub_odo = nh.subscribe("/bebop/odom", 30, odo_callback);
    pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
    pub_control = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,100);

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
            move_length(0.1); // 실험값 : 20~30 cm
            break;
          case 'y':
          case 'Y':
            move_angle(0.3); // 실험값 : 45도
            break;

        }
    }
    return 0;
}
