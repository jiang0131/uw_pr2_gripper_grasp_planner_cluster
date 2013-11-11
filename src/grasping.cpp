#include <uw_pr2_gripper_grasp_planner_cluster/grasping.h>

Grasping::Grasping():
  ac_planning("plan_point_cluster_grasp", true),
  ac_gripper("r_gripper_controller/gripper_action", true),
  group("right_arm")
{
  ROS_INFO("Waiting for GraspPlanningAction server to start.");
  ac_planning.waitForServer();
  ROS_INFO("GraspPlanningAction server is ready.");

  ROS_INFO("Waiting for Pr2GripperCommandAction server to start.");
  ac_gripper.waitForServer();
  ROS_INFO("Pr2GripperCommandAction server is ready.");

  sub_cluster = nh.subscribe("tabletop_cluster", 1000, &Grasping::pointcloud_callback, this);
  ros::spin();
};

Grasping::~Grasping() {
};


// Callback function when receiving a PointCloud2 message
void Grasping::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //
  std::cout << "THE PLANNING FRAME: " << group.getPlanningFrame() << std::endl;

  // Convert PointCloud2 to PointCloud
  sensor_msgs::PointCloud pc;
  convertPointCloud2ToPointCloud (*msg, pc);
  // Verify
  std::cout << "The size of the PointCloud:" << pc.points.size() << std::endl;

  // Open the gripper
  open_gripper();

  // Do grasp planning stuff
  manipulation_msgs::GraspableObject obj;
  obj.reference_frame_id = "base_link";
  obj.cluster = pc;

  // Fill GraspPlanningGoal
  //manipulation_msgs::GraspPlanningGoal goal;
  uw_pr2_gripper_grasp_planner_cluster::GraspPlanningGoal goal;
  goal.arm_name = "right_arm";
  goal.target = obj;

  // Send the goal to the grasp planning action server
  ac_planning.sendGoal(goal);
  // Wait for the action to return
  bool finished_before_timeout = ac_planning.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac_planning.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    uw_pr2_gripper_grasp_planner_cluster::GraspPlanningResult results = *ac_planning.getResult();

    // wait a bit for ros things to initialize
    ros::WallDuration(1.0).sleep();
    // Grasp the object called "part" (the name of the CollisionObject)
    group.pick("part", results.grasps);
  }
  else {
    ROS_INFO("Action did not finish before the time out.");
  }
};


void Grasping::open_gripper() {
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)
  ROS_INFO("Sending open goal");
  ac_gripper.sendGoal(open);
  ac_gripper.waitForResult();
  if(ac_gripper.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened!");
  else
    ROS_INFO("The gripper failed to open.");
};


int main(int argc, char **argv) {

  ros::init(argc, argv, "grasp_planning_client");
  
  Grasping grasping;

  //ros::NodeHandle n;
  
  // Action Client
  //ac = ac("plan_point_cluster_grasp", true);
  //ROS_INFO("Waiting for action server to start.");
  //ac.waitForServer();

  // Subscriber

  return 0;
}
