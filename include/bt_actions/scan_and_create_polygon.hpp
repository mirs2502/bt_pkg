#ifndef SCAN_AND_CREATE_POLYGON_HPP
#define SCAN_AND_CREATE_POLYGON_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace BT;

class ScanAndCreatePolygon : public SyncActionNode
{
public:
    ScanAndCreatePolygon(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            OutputPort<std::vector<geometry_msgs::msg::Point>>("polygon_map")
        };
    }

    NodeStatus tick() override {
        auto node = getInput<rclcpp::Node::SharedPtr>("node").value();
        RCLCPP_INFO(node->get_logger(), ">> [Phase 1] ポールを検出して地図を作成中...");
        
        // ダミーデータの出力
        std::vector<geometry_msgs::msg::Point> poly;
        setOutput("polygon_map", poly);
        
        // 処理時間を擬似的に作る
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return NodeStatus::SUCCESS;
    }
};
#endif