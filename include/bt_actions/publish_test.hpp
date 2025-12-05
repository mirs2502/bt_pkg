#ifndef PUBLISH_TEST_HPP
#define PUBLISH_TEST_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;

class PublishTest : public SyncActionNode
{
public:
    PublishTest(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config)
    {}

    // ポート定義：トピック名とメッセージ内容を受け取れるようにする
    static PortsList providedPorts()
    {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<std::string>("topic_name"),
            InputPort<std::string>("message_content")
        };
    }

    NodeStatus tick() override
    {
        // 1. ROSノードの取得
        auto node_ptr = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_ptr) {
            throw RuntimeError("Missing parameter [node] in PublishTest");
        }
        auto node = node_ptr.value();

        // 2. パラメータの取得
        std::string topic_name = "/chatter"; // デフォルト値
        std::string message_content = "Hello BT"; 
        
        getInput("topic_name", topic_name);
        getInput("message_content", message_content);

        // 3. パブリッシャーの作成（既に作成済みなら再利用するのが理想ですが、簡易実装としてここで作成）
        // ※本来はクラスのメンバ変数で保持する方が効率が良いですが、今回は分かりやすさ優先です
        auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name, 10);

        // 4. メッセージの作成と送信
        std_msgs::msg::String msg;
        msg.data = message_content;
        publisher->publish(msg);

        // ログにも出しておく
        RCLCPP_INFO(node->get_logger(), "Published to [%s]: %s", topic_name.c_str(), msg.data.c_str());

        return NodeStatus::SUCCESS;
    }
};

#endif // PUBLISH_TEST_HPP