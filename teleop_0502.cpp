#include <ros/ros.h>

// for getkey
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>

#include <cmath>  // for math

#include <sensor_msgs/NavSatFix.h> // for gps
#include <nav_msgs/Odometry.h>     // for odometry

geometry_msgs::Twist msg;
std_msgs::Empty empty;

ros::Publisher pub_cmd_vel;
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
void emergency()
{
  std::cout<<"| ### emergency ! ###"<<std::endl;
}
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
void key_info()
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
    std::cout<<"| g : move to gps point     "<<std::endl;
    std::cout<<"| p : emergency             "<<std::endl;
    std::cout<<"|                           "<<std::endl;
  }
}
void flight_info()
{
  
  std::cout <<"--------------------------------flight information--------------------------------"<< std::endl;
  if (curr_lat==500 && curr_lon==500 && curr_alt==500)
  std::cout <<"| gps         -> lat: ["<<curr_lat<<"], lon: ["<<curr_lon<<"], alt: ["<<curr_alt<<"]    ** GPS not connected **"<< std::endl;
  else
  std::cout <<"| gps         -> lat: ["<<curr_lat<<"], lon: ["<<curr_lon<<"], alt: ["<<curr_alt<<"]"<< std::endl;
  std::cout <<"| Position    -> x: ["<<pos_x<<"], y: ["<<pos_y<<"], z: ["<<pos_z<<"]"<< std::endl;
  std::cout <<"| Orientation -> x: ["<<ori_x<<"], y: ["<<ori_y<<"], z: ["<<ori_z<<"]"<< std::endl;
  std::cout <<"| Vel         -> Linear: ["<<vel_lin<<"], Angular: ["<<vel_ang<<"]"<< std::endl;
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

    ros::Subscriber sub_gps = nh.subscribe("/bebop/fix", 10, gps_callback);
    ros::Subscriber sub_odo = nh.subscribe("/bebop/odom", 30, odo_callback);

    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,100);
    pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);

    char key;
    
    while(ros::ok()){
      
      key = getKey();

      key_info();
      flight_info();

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
          case 'p':
          case 'P':
            emergency();
            break;
          case 'w':
          case 'W':
              msg.linear.x=0.3;
              msg.linear.y=0.0;
              msg.linear.z=0.0;
              msg.angular.x=0.0;
              msg.angular.y=0.0;
              msg.angular.z=0.0;
              pub_cmd_vel.publish(msg);
              break;
        }
    ros::spinOnce();
    ros::Rate(1).sleep();
    }

    return 0;
}