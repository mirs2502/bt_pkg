#ifndef RESET_CONES_HPP
#define RESET_CONES_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace BT;

class ResetCones : public StatefulActionNode
{
public:
    ResetCones(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<rclcpp::Node::SharedPtr>("node") };
    }

    NodeStatus onStart() override {
        auto node_res = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_res) {
            throw RuntimeError("Missing required input [node]");
        }
        node_ = node_res.value();

        // サービスクライアントの作成
        client_ = node_->create_client<std_srvs::srv::Trigger>("/reset_cones");

        // サービスが利用可能かチェック (非ブロッキングで確認したいが、初回だけ少し待つのは許容)
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "ResetCones: Service /reset_cones not available. Skipping reset.");
            return NodeStatus::SUCCESS; // サービスがなくてもミッションは止めない
        }

        // リクエスト送信
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        future_ = client_->async_send_request(request);
        
        RCLCPP_INFO(node_->get_logger(), "ResetCones: Service request sent.");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        // Futureの状態を確認
        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                auto result = future_.get();
                if (result->success) {
                    RCLCPP_INFO(node_->get_logger(), "ResetCones: %s", result->message.c_str());
                    return NodeStatus::SUCCESS;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "ResetCones: Service call failed: %s", result->message.c_str());
                    return NodeStatus::FAILURE;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "ResetCones: Exception during service call: %s", e.what());
                return NodeStatus::FAILURE;
            }
        }
        
        // まだ完了していない
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // キャンセル時の処理（特に必要なければ何もしない）
        RCLCPP_INFO(node_->get_logger(), "ResetCones: Halted.");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future_;
};
#endif
