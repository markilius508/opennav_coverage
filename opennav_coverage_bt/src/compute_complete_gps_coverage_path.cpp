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

#include "opennav_coverage_bt/compute_complete_gps_coverage_path.hpp"

namespace opennav_coverage_bt
{

// GPS Utility Functions for EPSG:4326
namespace gps_utils
{

// Convert degrees to radians
inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// Convert radians to degrees
inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

// Calculate great circle distance between two GPS points (Haversine formula)
// Returns distance in meters
double calculateGPSDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth's radius in meters
  
  double lat1_rad = deg2rad(lat1);
  double lat2_rad = deg2rad(lat2);
  double delta_lat = deg2rad(lat2 - lat1);
  double delta_lon = deg2rad(lon2 - lon1);
  
  double a = std::sin(delta_lat/2) * std::sin(delta_lat/2) +
             std::cos(lat1_rad) * std::cos(lat2_rad) *
             std::sin(delta_lon/2) * std::sin(delta_lon/2);
  double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
  
  return R * c;
}

// Linear interpolation between two GPS points
// t should be between 0.0 and 1.0
void interpolateGPS(double lat1, double lon1, double lat2, double lon2, 
                   double t, double& result_lat, double& result_lon) {
  // For short distances, linear interpolation in lat/lon is acceptable
  result_lat = lat1 + t * (lat2 - lat1);
  result_lon = lon1 + t * (lon2 - lon1);
}

// Great circle interpolation (more accurate for longer distances)
void interpolateGPSGreatCircle(double lat1, double lon1, double lat2, double lon2,
                              double t, double& result_lat, double& result_lon) {
  double lat1_rad = deg2rad(lat1);
  double lon1_rad = deg2rad(lon1);
  double lat2_rad = deg2rad(lat2);
  double lon2_rad = deg2rad(lon2);
  
  // Calculate angular distance
  double d_lat = lat2_rad - lat1_rad;
  double d_lon = lon2_rad - lon1_rad;
  double a = std::sin(d_lat/2) * std::sin(d_lat/2) + 
             std::cos(lat1_rad) * std::cos(lat2_rad) * 
             std::sin(d_lon/2) * std::sin(d_lon/2);
  double d = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
  
  if (d < 1e-6) {
    // Points are very close, use linear interpolation
    result_lat = lat1 + t * (lat2 - lat1);
    result_lon = lon1 + t * (lon2 - lon1);
    return;
  }
  
  double a_interp = std::sin((1-t) * d) / std::sin(d);
  double b_interp = std::sin(t * d) / std::sin(d);
  
  double x = a_interp * std::cos(lat1_rad) * std::cos(lon1_rad) + 
             b_interp * std::cos(lat2_rad) * std::cos(lon2_rad);
  double y = a_interp * std::cos(lat1_rad) * std::sin(lon1_rad) + 
             b_interp * std::cos(lat2_rad) * std::sin(lon2_rad);
  double z = a_interp * std::sin(lat1_rad) + b_interp * std::sin(lat2_rad);
  
  result_lat = rad2deg(std::atan2(z, std::sqrt(x*x + y*y)));
  result_lon = rad2deg(std::atan2(y, x));
}

} // namespace gps_utils

ComputeGPSCoveragePathAction::ComputeGPSCoveragePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf),
  max_distance_(20.0)
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

nav_msgs::msg::Path ComputeGPSCoveragePathAction::interpolateGPSPath(
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
    
    // Extract GPS coordinates (assuming x=longitude, y=latitude in EPSG:4326)
    double prev_lat = prev_pose.pose.position.y;
    double prev_lon = prev_pose.pose.position.x;
    double curr_lat = curr_pose.pose.position.y;
    double curr_lon = curr_pose.pose.position.x;
    
    // Calculate distance in meters
    double distance = gps_utils::calculateGPSDistance(prev_lat, prev_lon, curr_lat, curr_lon);
    
    if (distance > max_distance_meters) {
      // Calculate number of intermediate points needed
      const size_t num_intermediates = static_cast<size_t>(std::ceil(distance / max_distance_meters));
      const double step = 1.0 / (num_intermediates + 1);
      
      // Create intermediate points
      for (size_t j = 1; j <= num_intermediates; ++j) {
        const double ratio = j * step;
        
        double interp_lat, interp_lon;
        
        // Choose interpolation method based on distance
        if (distance < 1000.0) { // < 1km, use linear interpolation
          gps_utils::interpolateGPS(prev_lat, prev_lon, curr_lat, curr_lon, 
                                   ratio, interp_lat, interp_lon);
        } else { // >= 1km, use great circle interpolation
          gps_utils::interpolateGPSGreatCircle(prev_lat, prev_lon, curr_lat, curr_lon,
                                              ratio, interp_lat, interp_lon);
        }
        
        geometry_msgs::msg::PoseStamped intermediate;
        intermediate.header = input_path.header;
        intermediate.pose.position.x = interp_lon; // longitude
        intermediate.pose.position.y = interp_lat; // latitude
        intermediate.pose.position.z = prev_pose.pose.position.z; // maintain altitude
        intermediate.pose.orientation = prev_pose.pose.orientation; // maintain orientation
        
        interpolated_path.poses.push_back(intermediate);
      }
    }
    
    interpolated_path.poses.push_back(curr_pose);
  }
  
  return interpolated_path;
}

BT::NodeStatus ComputeGPSCoveragePathAction::on_success()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeGPSCoveragePath"), "Planning time: %.2f seconds", 
    result_.result->planning_time.sec + result_.result->planning_time.nanosec/1e9);

  // Apply GPS interpolation to the path with maximum distance [meters] between points
  // Try to get max_distance from behavior tree, use default (in header file) if not specified
  if (!getInput("max_distance", max_distance_)) {
    RCLCPP_INFO(
      rclcpp::get_logger("ComputeGPSCoveragePath"), 
      "max_distance not specified in BT, using default: %.1f meters", max_distance_);
  }
  nav_msgs::msg::Path interpolated_path = interpolateGPSPath(result_.result->nav_path, max_distance_);
  
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeGPSCoveragePath"), 
    "GPS interpolation completed: %zu poses after interpolation (original: %zu)", 
    interpolated_path.poses.size(), 
    result_.result->nav_path.poses.size()
  );

  // Calculate total path length for verification
  double total_distance = 0.0;
  for (size_t i = 1; i < interpolated_path.poses.size(); ++i) {
    const auto& prev_pose = interpolated_path.poses[i-1];
    const auto& curr_pose = interpolated_path.poses[i];
    total_distance += gps_utils::calculateGPSDistance(
      prev_pose.pose.position.y, prev_pose.pose.position.x,
      curr_pose.pose.position.y, curr_pose.pose.position.x
    );
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("ComputeGPSCoveragePath"), 
    "Total interpolated path length: %.2f meters", total_distance
  );

  // Set outputs with GPS-interpolated path
  setOutput("planning_time", result_.result->planning_time);
  setOutput("nav_path", interpolated_path); // Use GPS-interpolated path
  setOutput("coverage_path", result_.result->coverage_path);
  setOutput("error_code_id", ActionResult::NONE);

  // Blackboard debug
  auto blackboard = config().blackboard;
  RCLCPP_INFO(rclcpp::get_logger("ComputeGPSCoveragePath"), "Blackboard content:");
  
  nav_msgs::msg::Path path;
  if (blackboard->get<nav_msgs::msg::Path>("path", path)) {
    RCLCPP_INFO(rclcpp::get_logger("ComputeGPSCoveragePath"), "Path on blackboard has %zu poses", path.poses.size());
  }

  // Enhanced logging for GPS coordinates
  if (true) {
    std::ofstream logFile("/home/markilius/nav2_ws/src/gps_coverage_path_log.txt", std::ios::app);
    if (logFile.is_open()) {
        // Set precision to 10 decimal places and use fixed notation
        logFile << std::fixed << std::setprecision(10);
        
        // Log timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        logFile << "=== GPS Coverage Path Log - " << std::ctime(&time_t);
        
        logFile << "Original path: " << result_.result->nav_path.poses.size() << " poses\n";
        logFile << std::fixed << std::setprecision(2);
        logFile << "Interpolated path: " << interpolated_path.poses.size() << " poses" << " with max_distance: " << max_distance_ << "\n";
        logFile << "Total distance: " << total_distance << " meters\n";
        logFile << "GPS Coordinates (Longitude, Latitude):\n[";
        logFile << std::fixed << std::setprecision(10);
        for (size_t i = 0; i < interpolated_path.poses.size(); ++i) {
            const auto& pose = interpolated_path.poses[i];
            logFile << i << ":(lon=" << pose.pose.position.x << ", lat="
                    << pose.pose.position.y << ", alt=" << pose.pose.position.z << ")";
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