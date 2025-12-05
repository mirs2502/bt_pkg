#ifndef HELLO_ROS_HPP
#define HELLO_ROS_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

// 名前空間は必須ではないですが、あると便利です
using namespace BT;

// クラス定義：同期アクションノード（SyncActionNode）を継承
// ※ "Sync" は「すぐに終わる処理」用です。時間がかかる移動などは "Async" を使います。
class HelloRos : public SyncActionNode
{
public:
    // コンストラクタ（定型文）
    HelloRos(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config)
    {
    }

    // 1. ポートの定義
    // BlackboardにあるROSノードを受け取るための "node" ポートを定義します
    // さらに、XMLから任意のメッセージを受け取れるように "message" ポートも追加してみます
    static PortsList providedPorts()
    {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<std::string>("message")
        };
    }

    // 2. 実行時の処理（ここにやりたいことを書く！）
    NodeStatus tick() override
    {
        // (A) BlackboardからROSノードを取得
        auto node_ptr = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_ptr) {
            // 取得に失敗したらエラーを投げる（重要）
            throw RuntimeError("Missing parameter [node] in HelloRos");
        }
        rclcpp::Node::SharedPtr node = node_ptr.value();

        // (B) XMLからメッセージを取得（デフォルト値を設定しない場合はチェック推奨）
        std::string msg = "Hello World!"; // デフォルト
        auto msg_ptr = getInput<std::string>("message");
        if (msg_ptr) {
            msg = msg_ptr.value();
        }

        // (C) ROSの機能を使う（ここではログ出力）
        // node->create_publisher(...) などもここで書けます
        RCLCPP_INFO(node->get_logger(), "★ Action: %s", msg.c_str());

        // (D) 結果を返す
        // SUCCESS: 成功
        // FAILURE: 失敗
        return NodeStatus::SUCCESS;
    }
};

#endif // HELLO_ROS_HPP