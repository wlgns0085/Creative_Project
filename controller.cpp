#include <controller/controller.h>

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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
  pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
  pub_control = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,1);
  
  sub_pos = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 1, call_gps);
  sub_att = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, call_attitude);
  sub_bat = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, call_battery);
  sub_sat = nh.subscribe("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 1, call_satellite);

  _command* command;
  _compute* compute;
  _print* print;

  print->string("Controller Started");

  //while(ros::ok()){
  while(1){
    ros::spinOnce();
    
    char key = getKey();

    switch(key)
    //switch(drone.command)
    {
      case '1': // takeoff
        command->takeoff();
        print->command("takeoff");
        break;
      case '2': // land
        command->land();
        print->command("land");
        break;
      case '3': // stop
        command->stop();
        print->command("stop");
        break;
      case '4': // move to point ( lon, lat )
        command->goTo(drone.destination.longitude, drone.destination.latitude);
        print->command("move to poing");
        break;
      case '5': // flight plan ( route[] )
        command->flightplan(0.0);
        print->command("flight plan");
        break;
    }
  }
  print->string("Controller Exited");
  return 0;
}