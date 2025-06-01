#include "opennav_coverage_bt/convert_path_to_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"  // Add this include at the top of the file
#include <fstream>
#include "tf2/utils.h"

namespace opennav_coverage_bt
{

BT::NodeStatus ConvertPathToWaypoints::tick()
{
  auto logger = rclcpp::get_logger("ConvertPathToWaypoints");
  // Retrieve the input path from the blackboard
  nav_msgs::msg::Path path;
  if (!getInput("nav_path", path)) {
    // Return FAILURE if the input is missing or invalid
    RCLCPP_ERROR(logger, "Failed to get nav_path from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  // Log path information
  RCLCPP_INFO(logger, "Received nav_path with %zu poses, frame_id: %s", 
    path.poses.size(), path.header.frame_id.c_str());

  if (path.poses.empty()) {
  RCLCPP_WARN(logger, "Nav path contains no poses!");
  } else {
  // Log first and last position to verify data integrity
  RCLCPP_INFO(logger, "First pose position: x=%.2f, y=%.2f, z=%.2f",
        path.poses.front().pose.position.x,
        path.poses.front().pose.position.y,
        path.poses.front().pose.position.z);
        
  RCLCPP_INFO(logger, "Last pose position: x=%.2f, y=%.2f, z=%.2f",
        path.poses.back().pose.position.x,
        path.poses.back().pose.position.y,
        path.poses.back().pose.position.z);
  }

  if (!path.poses.empty()) {
    std::ofstream logFile("/home/markilius/nav2_ws/src/path_to_waypoints.txt", std::ios::app);
    if (logFile.is_open()) {
        logFile << "Received whole path with " << path.poses.size() << " poses: (Header.frame_id =)" << path.header.frame_id << "\n";
        logFile << "[";
        for (size_t i = 0; i < path.poses.size(); ++i) {
            const auto& pose = path.poses[i];
            logFile << i << ":(" << pose.pose.position.x << ", "
                    << pose.pose.position.y << ")";
            if (i < path.poses.size() - 1) {
                logFile << ", ";
            }
        }
        logFile << "]\n";
        logFile.close();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ConvertPathToWaypoints"), "Unable to open file for logging!");
    }
  }

  // Extract the poses from the path into a vector of waypoints
  std::vector<geometry_msgs::msg::PoseStamped> waypoints = path.poses;

  // Ensure all waypoints have the correct frame_id
  for (auto& waypoint : waypoints) {
    waypoint.header.frame_id = path.header.frame_id;
  }
  
  // Adding sampling logic (e.g., extracting every nth pose)
//   std::vector<geometry_msgs::msg::PoseStamped> sampled_waypoints;
//   for (size_t i = 0; i < path.poses.size(); i += 5) {
//     sampled_waypoints.push_back(path.poses[i]);
//   }
//   waypoints = sampled_waypoints;

  // Set the waypoints as an output on the blackboard
  setOutput("waypoints", waypoints);

  // Return SUCCESS to indicate the operation completed successfully
  return BT::NodeStatus::SUCCESS;
}

} // namespace opennav_coverage_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<opennav_coverage_bt::ConvertPathToWaypoints>("ConvertPathToWaypoints");
}