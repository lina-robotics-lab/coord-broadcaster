#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  while (!ros::isShuttingDown()) {
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    double temp_pose[7];
    std::cout << "Enter pose (x y z w x y z)" << std::endl;
    for(int i = 0; i < 7; i++)
    {
        std::cin >> temp_pose[i];
    }

    goal.target_pose.pose.position.x = temp_pose[0];
    goal.target_pose.pose.position.y = temp_pose[1];
    goal.target_pose.pose.position.z = temp_pose[2];
    goal.target_pose.pose.orientation.w = temp_pose[3];
    goal.target_pose.pose.orientation.x = temp_pose[4];
    goal.target_pose.pose.orientation.y = temp_pose[5];
    goal.target_pose.pose.orientation.z = temp_pose[6];

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved");
    else
      ROS_INFO("The base failed to move for some reason");

  }

  return 0;
}
