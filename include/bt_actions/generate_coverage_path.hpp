#ifndef GENERATE_COVERAGE_PATH_HPP
#define GENERATE_COVERAGE_PATH_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace BT;

class GenerateCoveragePath : public SyncActionNode
{
public:
    GenerateCoveragePath(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<std::vector<geometry_msgs::msg::Point>>("polygon_map"),
            OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("coverage_path")
        };
    }

    NodeStatus tick() override {
        auto node = getInput<rclcpp::Node::SharedPtr>("node").value();
        RCLCPP_INFO(node->get_logger(), ">> [Phase 2] カバレッジパス(ジグザグ経路)を計算中...");

        std::vector<geometry_msgs::msg::PoseStamped> path;
        setOutput("coverage_path", path);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return NodeStatus::SUCCESS;
    }
};
#endif