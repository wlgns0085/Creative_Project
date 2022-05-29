#include <controller/controller.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
  pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
  pub_control = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,1);
  pub_state = nh.advertise<std_msgs::Int16>("/bebop/state" ,1);
  

  ros::Subscriber sub_pos = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 1, call_gps);
  ros::Subscriber sub_att = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, call_attitude);
  ros::Subscriber sub_bat = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, call_battery);
  ros::Subscriber sub_sat = nh.subscribe("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 1, call_satellite);
  ros::Subscriber sub_command = nh.subscribe("/keyboard/command", 1, call_command);
  //ros::Subscriber sub_command = nh.subscribe("/Hololens/command", 1, call_command);

  std::cout<<"Controller Started"<<std::endl;

  while(ros::ok()){
    ros::spinOnce();
    publish_state();


    drone.destination.longitude = 36.14424294;
    drone.destination.latitude = 128.39389622;
    double route[4][2]={
    {36.14424294,128.39389622},
    {36.14424294,128.39375},
    {36.14415,128.39375},
    {36.14415,128.39389622}};
    

    switch(drone.command)
    {
      case 1: // takeoff
        std::cout<<"## Command : takeoff"<<std::endl;
        takeoff();
        change_state(2);
        drone.command = 0;
        break;
      case 2: // land
        std::cout<<"## Command : land"<<std::endl;
        land();
        change_state(1);
        drone.command = 0;
        break;
      case 3: // move to point ( lon, lat )
        std::cout<<"## Command : move to point"<<std::endl;
        change_state(3);
        goTo(drone.destination.longitude, drone.destination.latitude);
        change_state(2);
        if (emergency_check(drone.state)==1)
        {
          std::cout<<" !!! emergency occur !!!"<<std::endl;
          drone.state -= 90;
        }
        drone.command = 0;
        break;
      case 4: // flight plan ( route[] )
        std::cout<<"## Command : flight plan"<<std::endl;
        change_state(3);
        flightplan(route);
        change_state(2);
        if (emergency_check(drone.state)==1)
        {
          std::cout<<" !!! emergency occur !!!"<<std::endl;
          drone.state -= 90;
        }
        drone.command = 0;
        break;
      case 99: // emergency ( stop )
        std::cout<<"## Command : emergency"<<std::endl;
        drone.command = 0;
        break;
      case 9: // confirm safety
        std::cout<<"## Command : confirm safety"<<std::endl;
        if(drone.state/10==9) drone.state -= 90;
        drone.command = 0;
        break;
      case 81:
        std::cout<<"## Command : w"<<std::endl;
        control_lin(0.2);
        drone.command = 0;
        break;
      case 82:
        std::cout<<"## Command : s"<<std::endl;
        control_lin(-0.2);
        drone.command = 0;
        break;
      case 83:
        std::cout<<"## Command : a"<<std::endl;
        control_ang(0.2);
        drone.command = 0;
        break;
      case 84:
        std::cout<<"## Command : d"<<std::endl;
        control_ang(-0.2);
        drone.command = 0;
        break;
    }
  }
  std::cout<<("Controller Exited")<<std::endl;
  return 0;
}
