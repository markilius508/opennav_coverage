#ifndef CONVERT_PATH_TO_WAYPOINTS_HPP_
#define CONVERT_PATH_TO_WAYPOINTS_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace opennav_coverage_bt
{

class ConvertPathToWaypoints : public BT::SyncActionNode
{
public:
  // Constructor: Takes a node name and configuration
  ConvertPathToWaypoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  // Define the input and output ports
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("nav_path", "The input navigation path"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "The output waypoints"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("poses", "Destinations to plan through")
    };
  }

  // The main logic of the node, executed when ticked
  BT::NodeStatus tick() override;
};

} // namespace opennav_coverage_bt

#endif  // CONVERT_PATH_TO_WAYPOINTS_HPP_