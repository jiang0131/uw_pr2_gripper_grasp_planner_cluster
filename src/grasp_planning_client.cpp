#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <manipulation_msgs/GraspableObject.h>
#include <manipulation_msgs/GraspPlanningAction.h>

/*
class GraspPlanningClient {
public:
  GraspPlanningClient() {};
  ~GraspPlanningClient() {};

private:
}
*/

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert PointCloud2 to PointCloud
  sensor_msgs::PointCloud pc;
  convertPointCloud2ToPointCloud (*msg, pc);
  // Verify
  std::cout << "The size of the PointCloud:" << pc.points.size() << std::endl;
  
  // Do grasp planning stuff
  manipulation_msgs::GraspableObject obj;
  obj.reference_frame_id = "base_link";
  obj.cluster = pc;

  // Fill GraspPlanningGoal
  manipulation_msgs::GraspPlanningGoal goal;
  goal.arm_name = "right_arm";
  goal.target = obj;

  // Send the goal to the grasp planning action server
  actionlib::SimpleActionClient<manipulation_msgs::GraspPlanningAction> ac("plan_point_cluster_grasp", true);;
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server is ready.");

  ac.sendGoal(goal);
  // Wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    manipulation_msgs::GraspPlanningResult results = *ac.getResult();
    results.grasps //the planned grasps

  }
  else
    ROS_INFO("Action did not finish before the time out.");
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "grasp_planning_client");
  ros::NodeHandle n;
  
  // Action Client
  //ac = ac("plan_point_cluster_grasp", true);
  //ROS_INFO("Waiting for action server to start.");
  //ac.waitForServer();

  // Subscriber
  ros::Subscriber sub = n.subscribe("tabletop_cluster", 1000, pointcloud_callback);
  ros::spin();

  return 0;
}
