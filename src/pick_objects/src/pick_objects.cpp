#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv,  "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // ##### commands to pick  
  
  move_base_msgs::MoveBaseGoal roboPick;

  // set up the frame parameters
  roboPick.target_pose.header.frame_id = "map";
  roboPick.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  roboPick.target_pose.pose.position.x = -4.0;
  roboPick.target_pose.pose.position.y = -2.0;
  roboPick.target_pose.pose.orientation.z = 1.0;
  roboPick.target_pose.pose.orientation.w = 0.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick location");
  ac.sendGoal(roboPick);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pick location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {    
    ROS_INFO("Hooray, robo has started his journey to pick");
  	ros::Duration(5).sleep();
    ROS_INFO("Hooray, robo has picked up");
  	}
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  
   // ##### commands to drop
  
  move_base_msgs::MoveBaseGoal roboDrop;

  // set up the frame parameters
  roboDrop.target_pose.header.frame_id = "map";
  roboDrop.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  roboDrop.target_pose.pose.position.x = 4.4;
  roboDrop.target_pose.pose.position.y = -0.8;
  roboDrop.target_pose.pose.orientation.z = 0.0;
  roboDrop.target_pose.pose.orientation.w = 0.4;

   // Send the drop position and orientation for the robot to reach
  ROS_INFO("Sending drop location");
  ac.sendGoal(roboDrop);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pick location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, robo has started his journey to drop");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  
  return 0;
}