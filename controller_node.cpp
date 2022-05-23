#include <controller/controller.h>

/*

*/



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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
  pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
  pub_control = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel" ,1);

  //ros::Publisher pub_state = nh.advertise<state>("/bebop/state", 1);

  ros::Subscriber sub_pos = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 1, call_gps);
  ros::Subscriber sub_att = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, call_attitude);
  ros::Subscriber sub_bat = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, call_battery);
  ros::Subscriber sub_sat = nh.subscribe("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 1, call_satellite);


  std::cout<<"Controller Started"<<std::endl;
  std::cout<<"1. takeoff"<<std::endl;
  std::cout<<"2. land"<<std::endl;
  std::cout<<"3. stop"<<std::endl;
  std::cout<<"4. move to point"<<std::endl;
  std::cout<<"5. flight plan"<<std::endl;

  while(ros::ok()){
  //while(1){
    ros::spinOnce();
    
    std::cout<<"Wait for key : ";
    char key = getKey();
    drone.destination.longitude = 36.14424294;
    drone.destination.latitude = 128.39389622;

    double route[4][2]={
    {36.14424294,128.39389622},
    {36.14424294,128.39375},
    {36.14415,128.39375},
    {36.14415,128.39389622}};

    switch(key)
    //switch(drone.command)
    {
      case '0': // check info
        // Battery status
        std::cout<<std::right<<drone.battery<<"%  [";
        for (int i = 5; i >= (drone.battery + 10) / 20; i--) 
        {
          std::cout <<"□ ";
        }
        for (int i = 1; i < (drone.battery + 10) / 20; i++) 
        {
          std::cout <<"■ ";
        }
        std::cout <<"]"<<std::endl;
        std::cout <<"| GPS          -> lon: ["<<drone.longitude<<"], lat: ["<<drone.latitude<<"], alt: ["<<drone.altitude<<"]"<< std::endl;
        std::cout <<"| Attitude     -> yaw: ["<<drone.yaw<<"], pitch: ["<<drone.pitch<<"], roll: ["<<drone.roll<<"]"<< std::endl;
        break;
      case '1': // takeoff
        takeoff();
        std::cout<<"takeoff"<<std::endl;
        break;
      case '2': // land
        land();
        std::cout<<"land"<<std::endl;
        break;
      case '3': // stop
        stop();
        std::cout<<"stop"<<std::endl;
        break;
      case '4': // move to point ( lon, lat )
        std::cout<<"move to point"<<std::endl;
        goTo(drone.destination.longitude, drone.destination.latitude);
        break;
      case '5': // flight plan ( route[] )
        std::cout<<"flight plan"<<std::endl;
        flightplan(route);
        break;
    }
  }
  std::cout<<("Controller Exited")<<std::endl;
  return 0;
}