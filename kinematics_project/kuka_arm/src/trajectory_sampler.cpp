#include <trajectory_sampler.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_sampler");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*
   * Setup:
   * Load robot model, set planning_scene
   * Define the move_group for planning and control purpose
   */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  static const std::string PLANNING_GROUP = "arm_group";
  static const std::string GRIPPER_GROUP = "gripper_group";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface eef_group(GRIPPER_GROUP);

  // set RRT as the planner and set allowed planning time to 10 sec
  move_group.setPlannerId("RRTkConfigDefault");
  move_group.setPlanningTime(10.0);

  // set RRT as the planner and set allowed planning time to 10 sec
  eef_group.setPlannerId("RRTkConfigDefault");
  eef_group.setPlanningTime(5.0);

  //set tolerance off
  //ros::param::set("/move_group/trajectory_execution/allowed_start_tolerance", 0);
  //ros::param::set("/eef_group/trajectory_execution/allowed_start_tolerance", 0);

  // Define PlanningSceneInterface object to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Pointer to JointModelGroup for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const robot_state::JointModelGroup *gripper_joint_model_group =
    eef_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

  /*
   * Collision Objects:
   * Insert object for collision avoidance and interaction
   */
  moveit_msgs::CollisionObject shelf_collision_object, bin_collision_object;

  // add shelf collision object
  shelf_collision_object.header.frame_id = move_group.getPlanningFrame();
  shelf_collision_object.id = "shelf";

  shapes::Mesh* m = shapes::createMeshFromResource("package://kuka_arm/models/kinematics_shelf/kinematics_shelf.dae");

  ROS_INFO("Shelf mesh loaded");

  shape_msgs::Mesh shelf_mesh;
  shapes::ShapeMsg shelf_mesh_msg;
  shapes::constructMsgFromShape(m, shelf_mesh_msg);
  shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);
  shelf_collision_object.meshes.resize(1);
  shelf_collision_object.mesh_poses.resize(1);
  shelf_collision_object.meshes[0] = shelf_mesh;

  //Define a pose for the object (specified relative to base_footprint)
  shelf_collision_object.mesh_poses[0].position.x = 2.7;
  shelf_collision_object.mesh_poses[0].position.y = 0.0;
  shelf_collision_object.mesh_poses[0].position.z = 0.84;
  shelf_collision_object.mesh_poses[0].orientation.w= 0.707;
  shelf_collision_object.mesh_poses[0].orientation.x= 0.0;
  shelf_collision_object.mesh_poses[0].orientation.y= 0.0;
  shelf_collision_object.mesh_poses[0].orientation.z= 0.707;

  shelf_collision_object.meshes.push_back(shelf_mesh);
  shelf_collision_object.mesh_poses.push_back(shelf_collision_object.mesh_poses[0]);
  shelf_collision_object.operation = shelf_collision_object.ADD;

  // add bin collision object
  bin_collision_object.header.frame_id = move_group.getPlanningFrame();
  bin_collision_object.id = "bin";

  shapes::Mesh* n = shapes::createMeshFromResource("package://kuka_arm/models/kinematics_bin/kinematics_bin.dae");

  ROS_INFO("Bin mesh loaded");

  shape_msgs::Mesh bin_mesh;
  shapes::ShapeMsg bin_mesh_msg;
  shapes::constructMsgFromShape(n, bin_mesh_msg);
  bin_mesh = boost::get<shape_msgs::Mesh>(bin_mesh_msg);
  bin_collision_object.meshes.resize(1);
  bin_collision_object.mesh_poses.resize(1);
  bin_collision_object.meshes[0] = bin_mesh;

  //Define a pose for the object (specified relative to base_footprint)
  bin_collision_object.mesh_poses[0].position.x = 0.0;
  bin_collision_object.mesh_poses[0].position.y = 2.5;
  bin_collision_object.mesh_poses[0].position.z = 0.0;
  bin_collision_object.mesh_poses[0].orientation.w= 1.0;
  bin_collision_object.mesh_poses[0].orientation.x= 0.0;
  bin_collision_object.mesh_poses[0].orientation.y= 0.0;
  bin_collision_object.mesh_poses[0].orientation.z= 0.0;

  bin_collision_object.meshes.push_back(bin_mesh);
  bin_collision_object.mesh_poses.push_back(bin_collision_object.mesh_poses[0]);
  bin_collision_object.operation = bin_collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object_list;
  collision_object_list.push_back(shelf_collision_object);
  collision_object_list.push_back(bin_collision_object);

  // Add the object in world scene
  planning_scene_interface.addCollisionObjects(collision_object_list);
  ROS_INFO_NAMED("pick_place_project", "Added objects to the world");

  // Allow MoveGroup to add the collision object in the world
  ros::Duration(1.0).sleep();

  /*
   * rviz visualization:
   * Setup MoveItVisualTools for visualizing collision objects, robot,
   * and trajectories in Rviz as well as step-by-step progression
   */
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Load RemoteControl for step-by-step progression
  visual_tools.loadRemoteControl();

  // Create text marker for displaying current state
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 3.0;
  visual_tools.publishText(text_pose, "Welcome to pick-place project!",
    rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  // Publish messages to rviz
  visual_tools.trigger();
  visual_tools.prompt("next step");

  /*
   * Plan to target location:
   * We will get target position from param server and plan motion to the target
   */
  float target_x, target_y, target_z;
  float bin_x, bin_y, bin_z;

  ros::param::get("/target_spawn_location/x", target_x);
  ros::param::get("/target_spawn_location/y", target_y);
  ros::param::get("/target_spawn_location/z", target_z);

  ros::param::get("/target_drop_location/x", bin_x);
  ros::param::get("/target_drop_location/y", bin_y);
  ros::param::get("/target_drop_location/z", bin_z);

  geometry_msgs::Pose target_pose, bin_pose, target_reach;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = target_x-0.4;
  target_pose.position.y = target_y;
  target_pose.position.z = target_z-0.1;

  target_reach.orientation.w = 1.0;
  target_reach.position.x = target_x-0.2;
  target_reach.position.y = target_y;
  target_reach.position.z = target_z-0.1;

  bin_pose.orientation.w = 1.0;
  bin_pose.position.x = bin_x-0.1;
  bin_pose.position.y = bin_y;
  bin_pose.position.z = bin_z+1.6;

  // set starting pose
  move_group.setStartStateToCurrentState();

  // set target pose
  move_group.setPoseTarget(target_pose);

  //slow down movement of the robot
  move_group.setMaxVelocityScalingFactor(0.2);
  eef_group.setMaxVelocityScalingFactor(1.0);

  // define plan object which will contain the planned trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group.plan(my_plan);
  ROS_INFO_NAMED("pick_place_project", "Visualizing plan %s",
    success ? "SUCCESS" : "FAILED");

  // Visualize the plan
  visual_tools.publishAxisLabeled(target_pose, "pose1");
  visual_tools.publishText(text_pose, "Target Position",
    rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // command the robot to execute the created plan
  success = move_group.execute(my_plan);
  ROS_INFO_NAMED("pick_place_project", "Moving to pick location %s",
    success ? "SUCCESS" : "FAILED");
  visual_tools.prompt("next step");


  /*
   * Approach the target
   *
   */
  // set starting pose
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_reach);
  success = move_group.move();
  ROS_INFO_NAMED("pick_place_project", "Target reached %s",
    success ? "SUCCESS" : "FAILED");

  visual_tools.prompt("next step");

  /*
   * Grasp the target
   */
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr gripper_current_state = eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, joint_group_positions);

  //ROS_INFO_NAMED("pick_place_project", "size of: %d", joint_group_positions.size());
  
  joint_group_positions[0] = 0.02;  // radians
  joint_group_positions[1] = 0.02;  // radians
  eef_group.setJointValueTarget(joint_group_positions);

  success = eef_group.move();
  ROS_INFO_NAMED("pick_place_project", "Gripper actuation %s", success ? "SUCCESS" : "FAILED");
  visual_tools.prompt("next step");


  /*
   * Retract the gripper
   *
   */
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);
  success = move_group.move();
  ROS_INFO_NAMED("pick_place_project", "Target retrieved %s",
   success ? "SUCCESS" : "FAILED");

  visual_tools.prompt("next step");

  //plan to bin location for drop-off
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(bin_pose);

  success = move_group.plan(my_plan);
  ROS_INFO_NAMED("pick_place_project", "Visualizing plan 2 %s",
    success ? "SUCCESS" : "FAILED");

  // Visualize the plan
  visual_tools.publishAxisLabeled(bin_pose, "pose2");
  visual_tools.publishText(text_pose, "Moving to drop location",
    rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  //move_group.move();
  move_group.execute(my_plan);
  ROS_INFO_NAMED("pick_place_project", "Moving to drop location %s",
    success ? "SUCCESS" : "FAILED");
  visual_tools.prompt("next step");

  /*
   * Drop the target
   *
   */

  // RobotState is the object that contains all the current position/velocity/acceleration data.
  gripper_current_state = eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, joint_group_positions);

  //ROS_INFO_NAMED("pick_place_project", "size of: %d", joint_group_positions.size());
  
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = 0.0;  // radians
  eef_group.setJointValueTarget(joint_group_positions);

  success = eef_group.move();
  ROS_INFO_NAMED("pick_place_project", "Gripper actuation %s", success ? "SUCCESS" : "FAILED");

  ros::shutdown();
  return 0;
}
