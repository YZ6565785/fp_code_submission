#include <ros/ros.h>
#include <iostream>

// moveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_check");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  ROS_INFO("Default Pose Test:");
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  std::vector<std::pair<std::vector<double>, bool>> test_joint_values;
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false));
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, 1.03, 0.0, 0.0, 0.0, 0.0}, true));
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, 0.0, 0.19, 0.0, 0.0, 0.0}, true));
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, 0.0, 0.0, 0.0, 1.33, 0.0}, true));
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, 0.0, 0.0, 0.0, 1.31, 0.0}, true));
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, 0.0, 0.0, 0.0, 1.29, 0.0}, false));
  test_joint_values.push_back(std::make_pair(std::vector<double>{0.0, -1.05, -0.93, 0.0, 0.0, 0.0}, true));
  
  int i = 1;
  for (auto it = test_joint_values.begin(); it != test_joint_values.end(); it++)
  {
    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
    current_state.setVariablePositions(it->first);
    // current_state.printStatePositions(it->first);
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test: [" << i << "] " << (collision_result.collision == it->second? "PASSED" : "FAILED"));
    // ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    i++;
  }

  std::cout << "This is collision check node\t DONE." << std::endl;
  return 0;
}