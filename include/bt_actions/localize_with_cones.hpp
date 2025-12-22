#ifndef LOCALIZE_WITH_CONES_HPP
#define LOCALIZE_WITH_CONES_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using namespace BT;

class LocalizeWithCones : public StatefulActionNode
{
public:
    LocalizeWithCones(const std::string& name, const NodeConfiguration& config)
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

        // TFリスナーの初期化 (まだなければ)
        if (!tf_buffer_) {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        // Publisher
        pub_initialpose_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Subscriber
        landmark_received_ = false;
        sub_landmark_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/landmark_pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                last_landmark_pose_ = msg;
                landmark_received_ = true;
            });

        start_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "LocalizeWithCones: Waiting for /landmark_pose...");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        // タイムアウトチェック (3秒)
        if ((node_->now() - start_time_).seconds() > 3.0) {
            RCLCPP_WARN(node_->get_logger(), "LocalizeWithCones: Timeout. No landmark pose received.");
            return NodeStatus::SUCCESS; // 失敗させずに進む
        }

        if (landmark_received_ && last_landmark_pose_) {
            // 現在位置(TF)を取得
            geometry_msgs::msg::TransformStamped transform;
            try {
                // map -> base_link
                transform = tf_buffer_->lookupTransform(
                    "map", "base_link", tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "LocalizeWithCones: Could not get current robot pose: %s", ex.what());
                return NodeStatus::SUCCESS; // TF取れなければ諦める
            }

            // 距離チェック
            double dx = last_landmark_pose_->pose.pose.position.x - transform.transform.translation.x;
            double dy = last_landmark_pose_->pose.pose.position.y - transform.transform.translation.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            RCLCPP_INFO(node_->get_logger(), "LocalizeWithCones: Current Dist: %.2fm (Threshold: 1.0m)", dist);

            if (dist < 1.0) {
                // 妥当なのでリセット実行
                pub_initialpose_->publish(*last_landmark_pose_);
                RCLCPP_INFO(node_->get_logger(), "LocalizeWithCones: Published /initialpose to reset localization.");
            } else {
                RCLCPP_WARN(node_->get_logger(), "LocalizeWithCones: Landmark pose is too far (%.2fm). Ignoring.", dist);
            }

            // 少し待ってから終了
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return NodeStatus::SUCCESS;
        }

        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        sub_landmark_.reset();
        pub_initialpose_.reset();
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_landmark_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_landmark_pose_;
    bool landmark_received_;
    rclcpp::Time start_time_;
};
#endif