#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Pose RoboPick;
geometry_msgs::Pose RoboDrop;
bool roboStatus = true;
visualization_msgs::Marker marker;
visualization_msgs::Marker roboMarker;
ros::Publisher marker_pub;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    
    const bool roboPicking = (abs(RoboPick.position.x - msg->pose.pose.position.x) + abs(RoboPick.position.y - msg->pose.pose.position.y)) < 1;
    const bool roboDroping = (abs(RoboDrop.position.x - msg->pose.pose.position.x) + abs(RoboDrop.position.y - msg->pose.pose.position.y)) < 2;

    if(roboStatus){
        marker_pub.publish(marker);
     }

    if(roboPicking){
        roboStatus = false;
        sleep(5);
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
    	}
    else if(roboDroping){
        sleep(5);
        roboMarker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(roboMarker);
    }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  RoboPick.position.x = -0.5; // ###
  RoboPick.position.y = -7.0; // ###
  RoboDrop.position.x = -7.0; // ###
  RoboDrop.position.y = -5.5; // ###
  
  ros::Subscriber marker_sub = n.subscribe("/odom",1000,odomCallback);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE; // ###

  
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

 
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker.pose.position.x = RoboPick.position.x;
  marker.pose.position.y = RoboPick.position.y;

  roboMarker = marker;
  roboMarker.id = 1;
  roboMarker.pose.position.x = RoboDrop.position.x;
  roboMarker.pose.position.y = RoboDrop.position.y;

  marker_pub.publish(marker);

  while(ros::ok()){
      ros::spinOnce();
      r.sleep();
  }
}