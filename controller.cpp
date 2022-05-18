#include <controller/controller.h>

// ============== Compute ==================================================

void quaternion_to_euler(double x, double y, double z, double w) // 쿼터니언 to 오일러
{
  double t0 = 2.0 * (w*x+y*z);
  double t1 = 1.0 - 2.0*(x*x+y*y);
  roll_curr = atan2(t0,t1);
  double t2 = 2.0*(w*y-z*x);
  if(t2>1.0) t2=1.0;
  if(t2<-1.0) t2=-1.0;
  pitch_curr = asin(t2);
  double t3 = 2.0*(w*z+x*y);
  double t4 = 1.0 - 2.0*(y*y+z*z);
  yaw_curr = atan2(t3,t4);
}
double distance(double P1_latitude, double P1_longitude, double P2_latitude, double P2_longitude)
{
    if ((P1_latitude == P2_latitude) && (P1_longitude == P2_longitude))
    {
        return 0;
    }
    double e10 = P1_latitude * PI / 180;
    double e11 = P1_longitude * PI / 180;
    double e12 = P2_latitude * PI / 180;
    double e13 = P2_longitude * PI / 180;
    /* 타원체 GRS80 */
    double c16 = 6356752.314140910;
    double c15 = 6378137.000000000;
    double c17 = 0.0033528107;
    double f15 = c17 + c17 * c17;
    double f16 = f15 / 2;
    double f17 = c17 * c17 / 2;
    double f18 = c17 * c17 / 8;
    double f19 = c17 * c17 / 16;
    double c18 = e13 - e11;
    double c20 = (1 - c17) * tan(e10);
    double c21 = atan(c20);
    double c22 = sin(c21);
    double c23 = cos(c21);
    double c24 = (1 - c17) * tan(e12);
    double c25 = atan(c24);
    double c26 = sin(c25);
    double c27 = cos(c25);
    double c29 = c18;
    double c31 = (c27 * sin(c29) * c27 * sin(c29))
        + (c23 * c26 - c22 * c27 * cos(c29))
        * (c23 * c26 - c22 * c27 * cos(c29));
    double c33 = (c22 * c26) + (c23 * c27 * cos(c29));
    double c35 = sqrt(c31) / c33;
    double c36 = atan(c35);
    double c38 = 0;
    if (c31 == 0)
    {
        c38 = 0;
    }
    else
    {
        c38 = c23 * c27 * sin(c29) / sqrt(c31);
    }
    double c40 = 0;
    if ((cos(asin(c38)) * cos(asin(c38))) == 0)
    {
        c40 = 0;
    }
    else
    {
        c40 = c33 - 2 * c22 * c26
        / (cos(asin(c38)) * cos(asin(c38)));
    }
    double c41 = cos(asin(c38)) * cos(asin(c38))
        * (c15 * c15 - c16 * c16) / (c16 * c16);
    double c43 = 1 + c41 / 16384
        * (4096 + c41 * (-768 + c41 * (320 - 175 * c41)));
    double c45 = c41 / 1024 * (256 + c41 * (-128 + c41 * (74 - 47 * c41)));
    double c47 = c45
        * sqrt(c31)
        * (c40 + c45
        / 4
        * (c33 * (-1 + 2 * c40 * c40) - c45 / 6 * c40
            * (-3 + 4 * c31) * (-3 + 4 * c40 * c40)));
    double c50 = c17
        / 16
        * cos(asin(c38))
        * cos(asin(c38))
        * (4 + c17
        * (4 - 3 * cos(asin(c38))
            * cos(asin(c38))));
    double c52 = c18
        + (1 - c50)
        * c17
        * c38
        * (acos(c33) + c50 * sin(acos(c33))
        * (c40 + c50 * c33 * (-1 + 2 * c40 * c40)));
    double c54 = c16 * c43 * (atan(c35) - c47);
    // return distance in meter
    return c54;
}
short bearingP1toP2(double P1_latitude, double P1_longitude, double P2_latitude, double P2_longitude)
{
    // 현재 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 
    //라디안 각도로 변환한다.
    double Cur_Lat_radian = P1_latitude * (3.141592 / 180);
    double Cur_Lon_radian = P1_longitude * (3.141592 / 180);
    // 목표 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에
    // 라디안 각도로 변환한다.
    double Dest_Lat_radian = P2_latitude * (3.141592 / 180);
    double Dest_Lon_radian = P2_longitude * (3.141592 / 180);
    // radian distance
    double radian_distance = 0;
    radian_distance = acos(sin(Cur_Lat_radian)
        * sin(Dest_Lat_radian) + cos(Cur_Lat_radian)
        * cos(Dest_Lat_radian)
        * cos(Cur_Lon_radian - Dest_Lon_radian));
    // 목적지 이동 방향을 구한다.(현재 좌표에서 다음 좌표로 이동하기 위해서는 
    //방향을 설정해야 한다. 라디안값이다.
    double radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian)
        * cos(radian_distance))
        / (cos(Cur_Lat_radian) * sin(radian_distance)));
    // acos의 인수로 주어지는 x는 360분법의 각도가 아닌 radian(호도)값이다.
    double true_bearing = 0;
    if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0)
    {
        true_bearing = radian_bearing * (180 / 3.141592);
        true_bearing = 360 - true_bearing;
    }
    else
    {
        true_bearing = radian_bearing * (180 / 3.141592);
    }
    return true_bearing;
}
// ============== Subscriber ===============================================

void bat_callback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr data) // 베터리 잔량
{
  battery=data->percent;
}
void sat_callback(const bebop_msgs::Ardrone3GPSStateNumberOfSatelliteChanged::ConstPtr data) // 위성 연결
{
  sat_num = data->numberOfSatellite;
}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr data) // GPS 좌표
{
  lat_curr = (data->latitude);
  lon_curr = (data->longitude);
  alt_curr = (data->altitude);
}
void pos_callback(const bebop_msgs::Ardrone3PilotingStatePositionChanged::ConstPtr data) // GPS 좌표 상 변화
{
  lat_chng = data->latitude;
  lon_chng = data->longitude;
  alt_chng = data->altitude;
}
void odo_callback(const nav_msgs::Odometry::ConstPtr msg)  // Bebop 자세 
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
  lin_vel_curr = (msg->twist.twist.linear.x);
  ang_vel_curr = (msg->twist.twist.angular.z);

  quaternion_to_euler(ori_x, ori_y, ori_z, ori_w);
}
void att_callback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr data) // Bebop 자세 변화
{
  yaw_chng = (data->yaw)*180/PI;
  if (yaw_chng<0) yaw_chng+=360;
  pitch_chng = (data->pitch)*180/PI;
  roll_chng = (data->roll)*180/PI;
}
void spd_callback(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr data) // 방향에 따른 속도
{
  spd_x = data->speedX;
  spd_y = data->speedY;
  spd_z = data->speedZ;
}

// ============== Control ==================================================

void takeoff()
{
  state_takeoff = true;
  pub_takeoff.publish(empty);
}
void land()
{
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
void bearing_yaw(double target)
{
  double _yaw = yaw_chng;
  double _tar = target;
  if (_tar>=360) _tar -= 360;

  std::cout<<"target : "<<_tar<<std::endl;
  std::cout<<"current: "<<yaw_chng<<std::endl;

  int cw;
  double pre_yaw;
  if(_tar>yaw_chng)
  {
    if(_tar-yaw_chng>=180)
    {
      cw=+1;
      while(yaw_chng>_tar || yaw_chng<_tar-90)
      {
        control(0.0, 0.0, 0.0, cw*ang_vel);
        if(pre_yaw!=yaw_chng)
        {
          std::cout<<"cw: [ "<<cw<<" ] ";
          std::cout<<"target: [ "<<_tar<<" ] ";
          std::cout<<"current: [ "<<yaw_chng<<" ]"<<std::endl;
          pre_yaw=yaw_chng;
        }
        ros::spinOnce();
      }
    }
    else if (_tar-yaw_chng<180)
    {
      cw=-1;
      while(_tar>yaw_chng)
      {
        control(0.0, 0.0, 0.0, cw*ang_vel);
        if(pre_yaw!=yaw_chng)
        {
          std::cout<<"cw: [ "<<cw<<" ] ";
          std::cout<<"target: [ "<<_tar<<" ] ";
          std::cout<<"current: [ "<<yaw_chng<<" ]"<<std::endl;
          pre_yaw=yaw_chng;
        }
        ros::spinOnce();
      }
    }
  }
  else if(_tar<yaw_chng)
  {
    if(yaw_chng-_tar>=180)
    {
      cw=-1;
      while(yaw_chng<_tar || yaw_chng>_tar+90)
      {
        control(0.0, 0.0, 0.0, cw*ang_vel);
        if(pre_yaw!=yaw_chng)
        {
          std::cout<<"cw: [ "<<cw<<" ] ";
          std::cout<<"target: [ "<<_tar<<" ] ";
          std::cout<<"current: [ "<<yaw_chng<<" ]"<<std::endl;
          pre_yaw=yaw_chng;
        }
        ros::spinOnce();
      }
    }
    else if(yaw_chng-_tar<180)
    {
      cw=+1;
      while(_tar<yaw_chng)
      {
        control(0.0, 0.0, 0.0, cw*ang_vel);
        if(pre_yaw!=yaw_chng)
        {
          std::cout<<"cw: [ "<<cw<<" ] ";
          std::cout<<"target: [ "<<_tar<<" ] ";
          std::cout<<"current: [ "<<yaw_chng<<" ]"<<std::endl;
          pre_yaw=yaw_chng;
        }
        ros::spinOnce();
      }
    }
  }
  control(0.0, 0.0, 0.0, 0.0);
}
void compute_dest()
{
  ros::spinOnce();
  dest_dist = distance(lat_curr,lon_curr,lat_dest,lon_dest);
  dest_bear = bearingP1toP2(lat_curr,lon_curr,lat_dest,lon_dest);
}
void move_to_dest()
{
  double tmp_dest;
  compute_dest();
  bearing_yaw((int)dest_bear);
  tmp_dest=dest_bear;
  while(dest_dist>3)
  {
    ros::spinOnce();
    compute_dest();
    if(fabs(dest_bear-yaw_chng)>15) bearing_yaw((int)dest_bear);
    if(fabs(tmp_dest-dest_dist)>1)
    {
      std::cout<<"distance [ "<<dest_dist<<" ]"<<std::endl;
      std::cout<<"bearing  [ "<<dest_bear<<" ]"<<std::endl;
      tmp_dest=dest_dist;
    }
    control(lin_vel, 0.0, 0.0, 0.0);
  }
  control(0.0,0.0,0.0,0.0);
}


// ============== Main =====================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_gps = nh.subscribe("/bebop/fix", 1, gps_callback);
    ros::Subscriber sub_odo = nh.subscribe("/bebop/odom", 1, odo_callback);
    ros::Subscriber sub_bat = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, bat_callback);
    ros::Subscriber sub_pos = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 1, pos_callback);
    ros::Subscriber sub_att = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, att_callback);
    ros::Subscriber sub_spd = nh.subscribe("/bebop/states/ardrone3/PilotingState/SpeedChanged", 1, spd_callback);
    ros::Subscriber sub_sat = nh.subscribe("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 1, sat_callback);
    

    pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
    pub_control = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,1);

    lat_dest = 36.14424294;  // destination latitude
    lon_dest = 128.39389622; // destination longitude

    char key;

    while(ros::ok()){
      ros::spinOnce();

      print_simple_stat();
      print_stat();

      key = getKey();

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
            bearing_yaw(90);
            break;
          case 'y':
          case 'Y':
            std::cout<<"Put degree to rotate: ";
            int deg;
            std::cin>>deg;
            bearing_yaw(deg);
            break;

        }
    }
    return 0;
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