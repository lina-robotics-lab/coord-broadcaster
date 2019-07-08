#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


double poseAMCL[7];
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCL[0] = msgAMCL->pose.pose.position.x;
    poseAMCL[1] = msgAMCL->pose.pose.position.y;
    poseAMCL[2] = msgAMCL->pose.pose.position.z;
    poseAMCL[3] = msgAMCL->pose.pose.orientation.w;
    poseAMCL[4] = msgAMCL->pose.pose.orientation.x;
    poseAMCL[5] = msgAMCL->pose.pose.orientation.y;
    poseAMCL[6] = msgAMCL->pose.pose.orientation.z;

    //Give offset to the target pose
    for(int i = 0; i < 4; i++){
        ROS_INFO("%f", poseAMCL[i+3]);
    }

//    poseAMCL[1] = poseAMCL[1] - (double) 0.5;
//    ROS_INFO("%f", poseAMCL[1]);

    //ROS_INFO(msgAMCL);

}

int main(int argc, char** argv){


   ros::init(argc, argv, "target_broadcast");
   ros::NodeHandle n;

   ros::Subscriber sub_amcl = n.subscribe("/tb3_1/amcl_pose", 100, poseAMCLCallback);
   ros::Rate loop_rate(1);
   ros::spinOnce();

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("/tb3_0/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  while(ros::ok()){
      move_base_msgs::MoveBaseGoal goal;

      //we'll send a goal to the robot to move 1 meter forward
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = poseAMCL[0];
      goal.target_pose.pose.position.y = poseAMCL[1];
      goal.target_pose.pose.position.z = poseAMCL[2];
      goal.target_pose.pose.orientation.w = poseAMCL[3];
      goal.target_pose.pose.orientation.x = poseAMCL[4];
      goal.target_pose.pose.orientation.y = poseAMCL[5];
      goal.target_pose.pose.orientation.z = poseAMCL[6];

      ROS_INFO("Sending goal");

      ac.sendGoal(goal);

      ros::spinOnce();
      loop_rate.sleep();

//      ac.waitForResult();

//      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//        ROS_INFO("Hooray, the base moved");
//      else
//        ROS_INFO("The base failed to move for some reason");
  }

  return 0;
}
