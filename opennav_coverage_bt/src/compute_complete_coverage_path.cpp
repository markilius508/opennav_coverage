// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <chrono>

#include "opennav_coverage_bt/compute_complete_coverage_path.hpp"

namespace opennav_coverage_bt
{

// Cartesian Utility Functions
namespace cartesian_utils
{

// Calculate Euclidean distance between two Cartesian points
// Returns distance in meters (or coordinate units)
double calculateCartesianDistance(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

// Linear interpolation between two Cartesian points
// t should be between 0.0 and 1.0
void interpolateCartesian(double x1, double y1, double x2, double y2, 
                         double t, double& result_x, double& result_y) {
  result_x = x1 + t * (x2 - x1);
  result_y = y1 + t * (y2 - y1);
}

} // namespace cartesian_utils

ComputeCoveragePathAction::ComputeCoveragePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf),
  max_distance_(0.20)
{
}

void ComputeCoveragePathAction::on_tick()
{
  // Get core inputs about what to perform
  getInput("generate_headland", goal_.generate_headland);
  getInput("generate_route", goal_.generate_route);
  getInput("generate_path", goal_.generate_path);

  // Get the field to get coverage for
  std::string gml_filename;
  if (getInput("file_field", gml_filename)) {
    goal_.gml_field = gml_filename;
    goal_.use_gml_file = true;
  } else {
    getInput("polygons_frame_id", goal_.frame_id);

    // Convert from vector of Polygons to coverage specific message
    std::vector<geometry_msgs::msg::Polygon> polys;
    getInput("polygons", polys);
    goal_.polygons.resize(polys.size());
    for (unsigned int i = 0; i != polys.size(); i++) {
      for (unsigned int j = 0; j != polys[i].points.size(); j++) {
        opennav_coverage_msgs::msg::Coordinate coord;
        coord.axis1 = polys[i].points[j].x;
        coord.axis2 = polys[i].points[j].y;
        goal_.polygons[i].coordinates.push_back(coord);
      }
    }
  }
}

nav_msgs::msg::Path ComputeCoveragePathAction::interpolateCartesianPath(
  const nav_msgs::msg::Path& input_path, double max_distance_meters)
{
  nav_msgs::msg::Path interpolated_path;
  interpolated_path.header = input_path.header;
  
  if (input_path.poses.empty()) {
    return interpolated_path;
  }
  
  interpolated_path.poses.push_back(input_path.poses[0]);
  
  for (size_t i = 1; i < input_path.poses.size(); ++i) {
    const auto& prev_pose = input_path.poses[i-1];
    const auto& curr_pose = input_path.poses[i];
    
    // Extract Cartesian coordinates
    double prev_x = prev_pose.pose.position.x;
    double prev_y = prev_pose.pose.position.y;
    double curr_x = curr_pose.pose.position.x;
    double curr_y = curr_pose.pose.position.y;
    
    // Calculate distance in coordinate units
    double distance = cartesian_utils::calculateCartesianDistance(prev_x, prev_y, curr_x, curr_y);
    
    if (distance > max_distance_meters) {
      // Calculate number of intermediate points needed
      const size_t num_intermediates = static_cast<size_t>(std::ceil(distance / max_distance_meters));
      const double step = 1.0 / (num_intermediates + 1);
      
      // Create intermediate points using linear interpolation
      for (size_t j = 1; j <= num_intermediates; ++j) {
        const double ratio = j * step;
        
        double interp_x, interp_y;
        cartesian_utils::interpolateCartesian(prev_x, prev_y, curr_x, curr_y, 
                                             ratio, interp_x, interp_y);
        
        geometry_msgs::msg::PoseStamped intermediate;
        intermediate.header = input_path.header;
        intermediate.pose.position.x = interp_x;
        intermediate.pose.position.y = interp_y;
        intermediate.pose.position.z = prev_pose.pose.position.z; // maintain altitude
        intermediate.pose.orientation = prev_pose.pose.orientation; // maintain orientation
        
        interpolated_path.poses.push_back(intermediate);
      }
    }
    
    interpolated_path.poses.push_back(curr_pose);
  }
  
  return interpolated_path;
}

BT::NodeStatus ComputeCoveragePathAction::on_success()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeCoveragePath"), "Planning time: %.2f seconds", 
    result_.result->planning_time.sec + result_.result->planning_time.nanosec/1e9);

  // Apply Cartesian interpolation to the path with maximum distance between points
  // Try to get max_distance from behavior tree, use default (in header file) if not specified
  if (!getInput("max_distance", max_distance_)) {
    RCLCPP_INFO(
      rclcpp::get_logger("ComputeCoveragePath"), 
      "max_distance not specified in BT, using default: %.2f units", max_distance_);
  }
  nav_msgs::msg::Path interpolated_path = interpolateCartesianPath(result_.result->nav_path, max_distance_);
  
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeCoveragePath"), 
    "Cartesian interpolation completed: %zu poses after interpolation (original: %zu)", 
    interpolated_path.poses.size(), 
    result_.result->nav_path.poses.size()
  );

  // Calculate total path length for verification
  double total_distance = 0.0;
  for (size_t i = 1; i < interpolated_path.poses.size(); ++i) {
    const auto& prev_pose = interpolated_path.poses[i-1];
    const auto& curr_pose = interpolated_path.poses[i];
    total_distance += cartesian_utils::calculateCartesianDistance(
      prev_pose.pose.position.x, prev_pose.pose.position.y,
      curr_pose.pose.position.x, curr_pose.pose.position.y
    );
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeCoveragePath"), 
    "Total interpolated path length: %.2f units", total_distance
  );

  // Set outputs with Cartesian-interpolated path
  setOutput("planning_time", result_.result->planning_time);
  setOutput("nav_path", interpolated_path); // Use Cartesian-interpolated path
  setOutput("coverage_path", result_.result->coverage_path);
  setOutput("error_code_id", ActionResult::NONE);

  // Blackboard debug
  auto blackboard = config().blackboard;
  RCLCPP_INFO(rclcpp::get_logger("ComputeCoveragePath"), "Blackboard content:");
  
  nav_msgs::msg::Path path;
  if (blackboard->get<nav_msgs::msg::Path>("path", path)) {
    RCLCPP_INFO(rclcpp::get_logger("ComputeCoveragePath"), "Path on blackboard has %zu poses", path.poses.size());
  }

  // Enhanced logging for Cartesian coordinates
  if (true) {
    std::ofstream logFile("/home/markilius/nav2_ws/src/coverage_path_log.txt", std::ios::app);
    if (logFile.is_open()) {
        // Set precision to 10 decimal places and use fixed notation
        logFile << std::fixed << std::setprecision(10);
        
        // Log timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        logFile << "=== Cartesian Coverage Path Log - " << std::ctime(&time_t);
        
        logFile << "Original path: " << result_.result->nav_path.poses.size() << " poses\n";
        logFile << std::fixed << std::setprecision(3);
        logFile << "Interpolated path: " << interpolated_path.poses.size() << " poses" << " with max_distance: " << max_distance_ << "\n";
        logFile << "Total distance: " << total_distance << " units\n";
        logFile << "Cartesian Coordinates (X, Y):\n[";
        logFile << std::fixed << std::setprecision(10);
        
        for (size_t i = 0; i < interpolated_path.poses.size(); ++i) {
            const auto& pose = interpolated_path.poses[i];
            logFile << i << ":(x=" << pose.pose.position.x << ", y="
                    << pose.pose.position.y << ", z=" << pose.pose.position.z << ")";
            if (i < interpolated_path.poses.size() - 1) {
                logFile << ", ";
            }
            // Add newline every 5 points for readability
            if ((i + 1) % 5 == 0 && i < interpolated_path.poses.size() - 1) {
                logFile << "\n ";
            }
        }
        logFile << "]\n\n";
        logFile.close();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ComputeCoveragePath"), "Unable to open file for logging!");
    }
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeCoveragePathAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeCoveragePathAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeCoveragePathAction::halt()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  BtActionNode::halt();
}

}  // namespace opennav_coverage_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_coverage_bt::ComputeCoveragePathAction>(
        name, "compute_coverage_path", config);
    };

  factory.registerBuilder<opennav_coverage_bt::ComputeCoveragePathAction>(
    "ComputeCoveragePath", builder);
}