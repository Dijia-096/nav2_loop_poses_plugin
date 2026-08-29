#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_loop_poses_plugin/nav2_loop_poses_plugin.hpp"

namespace nav2_behavior_tree
{

LoopPoses::LoopPoses(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.7),
  loop_count_(0)
{
  getInput("radius", viapoint_achieved_radius_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("loop_count", loop_count_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

void LoopPoses::halt()
{
  // 中止/取消时清空运行时状态。该节点是同步 SUCCESS 节点，取消时通常不会被 halt 到，
  // 但作为防御性清理保留，避免任何复用场景下残留上一次的目标与圈数。
  original_input_goals_.clear();
  all_goals_.clear();
  original_goals_count_ = 0;
  step_count_ = 0;
  current_lap_ = 0;
}

bool LoopPoses::hasPassedWaypoint(
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & waypoint) const
{
  if (nav2_util::geometry_utils::euclidean_distance(waypoint.pose, current.pose) <=
    viapoint_achieved_radius_)
  {
    return true;
  }
  return false;
}

BT::NodeStatus LoopPoses::tick()
{
  Goals input_goals;
  getInput("input_goals", input_goals);

  // 检测新 goal（首次或抢占）：input_goals 与当前循环的原始快照不一致即视为新目标。
  // output 已改用独立键 {loop_window}，{goals} 始终是 navigator 写入的完整原始列表，
  // 故可直接据此判断，无需再依赖 all_goals_ 是否为空。
  if (input_goals != original_input_goals_) {
    original_input_goals_ = input_goals;
    all_goals_ = input_goals;
    original_goals_count_ = input_goals.size();
    step_count_ = 0;
    current_lap_ = 0;
  }

  if (all_goals_.empty()) {
    setOutput("output_goals", input_goals);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  const bool keep_looping = (loop_count_ == 0) || (current_lap_ < loop_count_);

  // 去掉已贴近的途经点：仅按半径判断。
  // 每 tick 最多切一整圈（original_goals_count_ 次），防止所有点都判定为已过时无限旋转。
  size_t removed = 0;
  while (all_goals_.size() > 1 && removed < original_goals_count_) {
    if (!hasPassedWaypoint(current_pose, all_goals_[0])) {
      break;
    }
    if (keep_looping) {
      all_goals_.push_back(all_goals_[0]);
      step_count_++;
      if (step_count_ % original_goals_count_ == 0) {
        current_lap_++;
      }
    }
    all_goals_.erase(all_goals_.begin());
    removed++;
  }

  // 有限圈数下，完成所有圈后输出空，导航结束
  if (loop_count_ != 0 && current_lap_ >= loop_count_) {
    setOutput("output_goals", Goals{});
    return BT::NodeStatus::SUCCESS;
  }

  // 输出到黑板：只输出当前目标点 + 其下一个点（2 点窗口），实现无缝循环
  Goals output;
  output.reserve(2);
  output.push_back(all_goals_.front());
  if (all_goals_.size() >= 2) {
    output.push_back(all_goals_[1]);
  }
  setOutput("output_goals", output);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::LoopPoses>("LoopPoses");
}
