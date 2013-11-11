#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <manipulation_msgs/GraspableObject.h>
//#include <manipulation_msgs/GraspPlanningAction.h>
#include <uw_pr2_gripper_grasp_planner_cluster/GraspPlanningAction.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/Grasp.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

class Grasping {
public:
  Grasping();
  ~Grasping();

private:

  ros::NodeHandle nh;
  // Subscriber for tabletop cluster PointCloud2
  ros::Subscriber sub_cluster;
  // SimpleActionClient for GraspPlanningAction  
  actionlib::SimpleActionClient<uw_pr2_gripper_grasp_planner_cluster::GraspPlanningAction> ac_planning;
  // SimpleActionClient for pr2_controllers_msgs::Pr2GripperCommandAction  
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> ac_gripper;
  // MoveIt! MoveGroupInterface
  moveit::planning_interface::MoveGroup group;

  // Callback function when receiving a PointCloud2 message
  void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  // Open the gripper
  void open_gripper();

  // Pick up a CollisionObject in PlanningScene using the given Grasp[]
  //void pick(const& string obj_name, const Grasps& grasps);

};
