#ifndef LOCALIZE_WITH_CONES_HPP
#define LOCALIZE_WITH_CONES_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

using namespace BT;

class LocalizeWithCones : public SyncActionNode
{
public:
    LocalizeWithCones(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<rclcpp::Node::SharedPtr>("node") };
    }

    NodeStatus tick() override {
        auto node = getInput<rclcpp::Node::SharedPtr>("node").value();
        RCLCPP_WARN(node->get_logger(), "!! [Recovery] コーンを使って自己位置を補正中 !!");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return NodeStatus::SUCCESS;
    }
};
#endif