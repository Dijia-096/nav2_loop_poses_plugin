#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CUSTOM__LOOP_POSES_PLUGIN_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CUSTOM__LOOP_POSES_PLUGIN_HPP_

#include <vector>
#include <memory>
#include <string>
#include <cstdint>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class LoopPoses : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  LoopPoses(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to loop through"),
      BT::OutputPort<Goals>("output_goals", "Goals to loop through"),
      BT::InputPort<double>("radius", 0.7, "Radius tolerance on a goal to consider it passed"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<std::int16_t>("loop_count", 0,
        "Number of laps to complete. 0 = infinite loop."),
    };
  }

private:
  void halt() override;
  BT::NodeStatus tick() override;

  // 判断机器人是否已贴近途经点 waypoint（仅半径判断）
  bool hasPassedWaypoint(
    const geometry_msgs::msg::PoseStamped & current,
    const geometry_msgs::msg::PoseStamped & waypoint) const;

  // 配置参数
  double viapoint_achieved_radius_;
  std::string robot_base_frame_, global_frame_;
  double transform_tolerance_;
  std::int16_t loop_count_;     // 0=无限循环, N=完成N圈后停止

  // 运行时状态
  Goals original_input_goals_;       // 当前循环对应的原始目标快照（用于检测新 goal / 抢占）
  Goals all_goals_;                  // 循环中的工作副本（每 tick 在其上推进）
  size_t original_goals_count_{0};   // 初始 goals 长度
  size_t step_count_{0};             // 累计切点次数
  int current_lap_{0};               // 当前已完成的圈数

  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CUSTOM__LOOP_POSES_PLUGIN_HPP_
