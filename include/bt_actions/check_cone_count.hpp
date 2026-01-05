#ifndef CHECK_CONE_COUNT_HPP
#define CHECK_CONE_COUNT_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

using namespace BT;

class CheckConeCount : public ConditionNode
{
public:
    CheckConeCount(const std::string& name, const NodeConfiguration& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<int>("target_count", 4, "Number of cones required to return SUCCESS")
        };
    }

    NodeStatus tick() override {
        // ノードの取得 (Blackboardからではなく、静的変数やシングルトン、あるいはこのノード自身がROSノードを持つ設計も考えられるが、
        // ここでは既存の設計に合わせてBlackboardからROSノードを取得し、サブスクリプションを行う...
        // しかし、ConditionNodeはtickのたびに呼ばれるため、サブスクリプションをここで作るのは不適切。
        // 簡易的な実装として、グローバル変数や静的メンバでデータを共有するか、
        // あるいはこのクラス自体がROSノードの機能を持つ必要がある。
        // 
        // 既存の bt_pkg の設計を見ると、ActionNode は onStart でサブスクライブしているものが多い。
        // ConditionNode は tick() しかないため、初回 tick でサブスクライブを開始し、
        // コールバックでメンバ変数を更新する形にするのが良い。
        
        if (!node_) {
            auto node_res = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            if (!node_res) {
                // まだノードがセットされていない（ありえないはずだが）
                return NodeStatus::FAILURE;
            }
            node_ = node_res;
            
            // サブスクライバの作成
            sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/accumulated_cones", 10,
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    current_cone_count_ = msg->width * msg->height; // PointCloud2の点数
                    // RCLCPP_INFO(node_->get_logger(), "CheckConeCount: Current cones: %d", current_cone_count_);
                });
            
            RCLCPP_INFO(node_->get_logger(), "CheckConeCount initialized. Waiting for cones...");
        }

        int target_count = 4;
        getInput("target_count", target_count);

        if (current_cone_count_ >= target_count) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
                "CheckConeCount: Target reached! (%d >= %d)", current_cone_count_, target_count);
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
                "CheckConeCount: Not enough cones (%d < %d)", current_cone_count_, target_count);
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    int current_cone_count_ = 0;
};

#endif
