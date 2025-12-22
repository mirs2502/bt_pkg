#ifndef GENERATE_COVERAGE_PATH_HPP
#define GENERATE_COVERAGE_PATH_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace BT;

class GenerateCoveragePath : public StatefulActionNode
{
public:
    GenerateCoveragePath(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<std::vector<geometry_msgs::msg::Point>>("area_polygon"),
            OutputPort<nav_msgs::msg::Path>("coverage_path"),
            OutputPort<geometry_msgs::msg::PoseStamped>("start_pose")
        };
    }

    NodeStatus onStart() override {
        auto node_res = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_res) {
            throw RuntimeError("Missing required input [node]");
        }
        node_ = node_res.value();
        
        // Subscribe to /coverage_path
        sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "/coverage_path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                RCLCPP_INFO(node_->get_logger(), "GenerateCoveragePath: Path received with %zu poses.", msg->poses.size());
                last_path_ = msg;
            });

        start_time_ = node_->now(); // 開始時刻を記録
        RCLCPP_INFO(node_->get_logger(), "Waiting for path from /coverage_path...");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        if (last_path_) {
            setOutput("coverage_path", *last_path_);
            
            if (!last_path_->poses.empty()) {
                setOutput("start_pose", last_path_->poses[0]);
            }

            return NodeStatus::SUCCESS;
        }

        // タイムアウトチェック (5秒)
        auto current_time = node_->now();
        if ((current_time - start_time_).seconds() > 5.0) {
            RCLCPP_ERROR(node_->get_logger(), "GenerateCoveragePath: Timeout waiting for path.");
            return NodeStatus::FAILURE;
        }

        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        sub_.reset();
        last_path_.reset(); // リセット時にデータもクリア
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
    nav_msgs::msg::Path::SharedPtr last_path_;
    rclcpp::Time start_time_; // タイムアウト計測用
};
#endif