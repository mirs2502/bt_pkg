#ifndef NAV2_MOCK_NODES_HPP
#define NAV2_MOCK_NODES_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // ★追加

using namespace BT;

// Waitの代用 (変更なし)
class Wait : public SyncActionNode {
public:
    Wait(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
    static PortsList providedPorts() { return { InputPort<double>("wait_duration") }; }
    NodeStatus tick() override {
        double duration = 1.0;
        if (getInput("wait_duration", duration)) {
            std::cout << "[Wait Node] Waiting for " << duration << " seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));
            return NodeStatus::SUCCESS;
        } else {
             throw RuntimeError("Missing parameter [wait_duration]");
        }
    }
};

// Spinの代用 (変更なし)
class Spin : public SyncActionNode {
public:
    Spin(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
    static PortsList providedPorts() { return { InputPort<double>("spin_dist") }; }
    NodeStatus tick() override {
        std::cout << "[Nav2 Mock] Spinning..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return NodeStatus::SUCCESS;
    }
};

// NavigateThroughPosesの代用 (★ここを修正)
class NavigateThroughPoses : public SyncActionNode {
public:
    NavigateThroughPoses(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
    
    // ★修正: ポートの型を std::string から PoseStampedのvector に変更
    static PortsList providedPorts() { 
        return { InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals") }; 
    }
    
    NodeStatus tick() override {
        // 入力の取得確認（エラーチェック）
        auto goals_ptr = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
        if (!goals_ptr) {
            throw RuntimeError("Missing parameter [goals] in NavigateThroughPoses");
        }

        // ランダムに失敗させてリカバリー動作を確認する
        if (rand() % 2 == 0) {
            std::cout << "[Nav2 Mock] Navigation Failed! -> Switching to Recovery" << std::endl;
            return NodeStatus::FAILURE;
        }
        std::cout << "[Nav2 Mock] Navigation Success!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return NodeStatus::SUCCESS;
    }
};
#endif