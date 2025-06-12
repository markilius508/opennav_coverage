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

#include "opennav_coverage_bt/compute_complete_gps_coverage_path.hpp"

namespace opennav_coverage_bt
{

ComputeGPSCoveragePathAction::ComputeGPSCoveragePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ComputeGPSCoveragePathAction::on_tick()
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

    // Convert from vector of Polygons to coverage sp. message
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

BT::NodeStatus ComputeGPSCoveragePathAction::on_success()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeGPSCoveragePath"), "Planning time: %.2f seconds", 
    result_.result->planning_time.sec + result_.result->planning_time.nanosec/1e9);

  // Create filtered path
  nav_msgs::msg::Path filtered_path;
  filtered_path.header = result_.result->nav_path.header;
  
  // First pass: Filter poses with y <= 6.0
  for (const auto& pose : result_.result->nav_path.poses) {
    if (pose.pose.position.y <= 20.0) {
      filtered_path.poses.push_back(pose);
    }
  }

  // Second pass: Interpolate between distant poses
  nav_msgs::msg::Path interpolated_path;
  interpolated_path.header = filtered_path.header;
  
  if (!filtered_path.poses.empty()) {
    interpolated_path.poses.push_back(filtered_path.poses[0]);
    
    for (size_t i = 1; i < filtered_path.poses.size(); ++i) {
      const auto& prev_pose = filtered_path.poses[i-1];
      const auto& curr_pose = filtered_path.poses[i];
      
      const double dx = curr_pose.pose.position.x - prev_pose.pose.position.x;
      const double dy = curr_pose.pose.position.y - prev_pose.pose.position.y;
      const double distance = std::hypot(dx, dy);

      double max_distance = 0.20;
      
      if (distance > max_distance) {
        // Calculate number of intermediate points needed
        const size_t num_intermediates = static_cast<size_t>(std::ceil(distance / max_distance));
        const double step = 1.0 / (num_intermediates + 1);
        
        // Linear interpolation
        for (size_t j = 1; j <= num_intermediates; ++j) {
          const double ratio = j * step;
          geometry_msgs::msg::PoseStamped intermediate;
          intermediate.header = filtered_path.header;
          intermediate.pose.position.x = prev_pose.pose.position.x + (dx * ratio);
          intermediate.pose.position.y = prev_pose.pose.position.y + (dy * ratio);
          intermediate.pose.orientation = prev_pose.pose.orientation; // Maintain orientation
          interpolated_path.poses.push_back(intermediate);
        }
      }
      
      interpolated_path.poses.push_back(curr_pose);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ComputeGPSCoveragePath"), 
    "Processed path: %zu poses after filtering and interpolation (original: %zu)", 
    interpolated_path.poses.size(), 
    result_.result->nav_path.poses.size()
  );

  // Set outputs with processed path
  setOutput("planning_time", result_.result->planning_time);
  setOutput("nav_path", result_.result->nav_path);
  // setOutput("nav_path", interpolated_path);
  setOutput("coverage_path", result_.result->coverage_path);
  setOutput("error_code_id", ActionResult::NONE);

  // Blackboard debug
  auto blackboard = config().blackboard;
  RCLCPP_INFO(rclcpp::get_logger("ComputeGPSCoveragePath"), "Blackboard content:");
  
  nav_msgs::msg::Path path;
  if (blackboard->get<nav_msgs::msg::Path>("path", path)) {
    RCLCPP_INFO(rclcpp::get_logger("ComputeGPSCoveragePath"), "Path on blackboard has %zu poses", path.poses.size());
  }

  if (true) {
    std::ofstream logFile("/home/markilius/nav2_ws/src/gps_coverage_path_log.txt", std::ios::app);
    if (logFile.is_open()) {
        // Set precision to 10 decimal places and use fixed notation
        logFile << std::fixed << std::setprecision(10);
        
        logFile << "Received path with " << path.poses.size() << " poses:\n";
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
        RCLCPP_ERROR(rclcpp::get_logger("ComputeGPSCoveragePath"), "Unable to open file for logging!");
    }
}
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeGPSCoveragePathAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeGPSCoveragePathAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  opennav_coverage_msgs::msg::PathComponents cov_path;
  setOutput("nav_path", empty_path);
  setOutput("coverage_path", cov_path);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeGPSCoveragePathAction::halt()
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
      return std::make_unique<opennav_coverage_bt::ComputeGPSCoveragePathAction>(
        name, "compute_coverage_path", config);
    };

  factory.registerBuilder<opennav_coverage_bt::ComputeGPSCoveragePathAction>(
    "ComputeGPSCoveragePath", builder);
}
