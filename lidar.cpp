#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <math.h>

ros::Publisher marker_pub;
void Pub_point(float distance, int angle);
void Pub_point_zero();
void Pub_arrow(float angle);

#define arr_size 16
int arr_index = 0;
float point_arr[2][16];
int triger = 0;


float distance;
int angle;

void call_distance(const std_msgs::Float64::ConstPtr data) // 거리
{
  distance = data->data;
}
void call_angle(const std_msgs::Int16::ConstPtr data) // 각도
{
  angle = data->data;
  printf("degree [ %d ] / dist : [ %f ]\n",angle, distance);
  Pub_point(distance/2,angle * 3.141592 / 180);

  point_arr[0][arr_index]=distance;
  point_arr[1][arr_index]=angle*1.0;

  arr_index++;
  if(arr_index==arr_size) {arr_index = 0; triger=1;}
   
  if(triger==1){
    for(int i=0;i<arr_size;i++){}
      //Pub_point(point_arr[0][i]/2,point_arr[1][i] * 3.141592 / 180);
  }
  
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle n;

    ros::Subscriber sub_distance = n.subscribe("/bebop/distance", 1, call_distance);
    ros::Subscriber sub_angle = n.subscribe("/bebop/angle", 1, call_angle);
    marker_pub = n.advertise<visualization_msgs::Marker>("/lidar/marker", 10);
  
    ros::Rate r(120);

    while (ros::ok())
    {
      //Pub_point(distance, angle);
      
      Pub_point_zero();
      //Pub_arrow(angle);


    ros::spinOnce();
  }
}
void Pub_point(float distance, int angle)
{
    visualization_msgs::Marker points;

    points.header.frame_id = "my_frame";
    points.header.stamp = ros::Time::now();
    points.ns="point";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::SPHERE;
    
    points.pose.position.x = distance *sin(angle);
    points.pose.position.y = distance *cos(angle);
    points.pose.position.z = 0;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.scale.z = 0.5;
    points.color.a = 1.0; // Don't forget to set the alpha!
    points.color.r = 1.0;
    points.color.g = 0.0;
    points.color.b = 0.0;
	
	marker_pub.publish( points );
}
void Pub_point_zero()
{
    visualization_msgs::Marker points;

    points.header.frame_id = "my_frame";
    points.header.stamp = ros::Time::now();
    points.ns="zero";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::CUBE;
    
    points.pose.position.x = 0;
    points.pose.position.y = 0;
    points.pose.position.z = 0;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.scale.z = 0.5;
    points.color.a = 1.0; // Don't forget to set the alpha!
    points.color.r = 0.0;
    points.color.g = 1.0;
    points.color.b = 1.0;
	
	  marker_pub.publish( points );
}
void Pub_arrow(float angle)
{
    visualization_msgs::Marker points;

    points.header.frame_id = "my_frame";
    points.header.stamp = ros::Time::now();
    points.ns="arrow_namespace";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::ARROW;
    
    points.pose.position.x = 0;
    points.pose.position.y = 0;
    points.pose.position.z = 1;
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = angle;
    points.pose.orientation.w = 1.0;
    points.scale.x = 5;
    points.scale.y = 0.5;
    points.scale.z = 0.2;
    points.color.a = 1.0; // Don't forget to set the alpha!
    points.color.r = 0.0;
    points.color.g = 1.0;
    points.color.b = 0.0;
	
	marker_pub.publish( points );
}