#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h" // Groot用
#include "bt_actions/hello_ros.hpp"
#include "bt_actions/publish_test.hpp"

// ここに自作ノードのヘッダをインクルードしていく
#include "bt_actions/scan_and_create_polygon.hpp"
#include "bt_actions/generate_coverage_path.hpp"
#include "bt_actions/localize_with_cones.hpp"
#include "bt_actions/control_motor.hpp" // Added
#include "bt_actions/drive_robot.hpp" // Added
//Nav2関連
#include <filesystem>
#include <string>
#include <vector>
#include "bt_actions/nav2_mock_nodes.hpp" // モック


// XMLファイルのパスを定数で定義（実際はパラメータやLaunchで渡すと良い）
// 重要: フルパスで指定するか、shareディレクトリから探すロジックが必要

#include <csignal>
std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);

    // 1. 共有ROSノードの作成
    auto ros_node = rclcpp::Node::make_shared("bt_executor_node");

    // パラメータの宣言と取得
    ros_node->declare_parameter<std::string>("bt_xml_path", "");
    std::string bt_xml_path;
    ros_node->get_parameter("bt_xml_path", bt_xml_path);

    if (bt_xml_path.empty()) {
        RCLCPP_ERROR(ros_node->get_logger(), "Parameter 'bt_xml_path' is not set!");
        return 1;
    }

    std::cout << "DEBUG: Loading XML from: " << bt_xml_path << std::endl;

    // 2. Factoryの作成
    BT::BehaviorTreeFactory factory;

    // ここで自作ノードを登録する
    factory.registerNodeType<ScanAndCreatePolygon>("ScanAndCreatePolygon");
    factory.registerNodeType<GenerateCoveragePath>("GenerateCoveragePath");
    factory.registerNodeType<LocalizeWithCones>("LocalizeWithCones");
    factory.registerNodeType<ControlMotor>("ControlMotor"); // Added
    factory.registerNodeType<DriveRobot>("DriveRobot"); // Added

    // Nav2モックノードの登録 (テスト用)
    factory.registerNodeType<Wait>("Wait");
    //factory.registerNodeType<Spin>("Spin");
    //factory.registerNodeType<NavigateThroughPoses>("NavigateThroughPoses");

    
    const std::string plugin_dir = "/opt/ros/humble/lib"; // ROS2の標準ライブラリパス
    namespace fs = std::filesystem;

    try {
        for (const auto & entry : fs::directory_iterator(plugin_dir)) {
            std::string filename = entry.path().filename().string();
            if (filename.find("_bt_node.so") != std::string::npos) {
                if (filename.find("wait") != std::string::npos) {
                    continue;
                }
                factory.registerFromPlugin(entry.path().string());
                RCLCPP_INFO(ros_node->get_logger(), "Loaded plugin: %s", filename.c_str());
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(ros_node->get_logger(), "Failed to load plugins: %s", e.what());
    }
    

    factory.registerNodeType<HelloRos>("HelloRos");
    factory.registerNodeType<PublishTest>("PublishTest");

    // 3. Blackboardの設定とROSノードの登録
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", ros_node); 
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10000)); 
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(10000)); 

    // 4. ツリーの構築
    BT::Tree tree;
    bool tree_created = false;
    int max_retries = 20; // 20 retries * 1s = 20s (plus 5s initial delay = 25s total)
    
    for (int i = 0; i < max_retries; ++i) {
        if (g_shutdown_requested) break;
        try {
            tree = factory.createTreeFromFile(bt_xml_path, blackboard);
            tree_created = true;
            break;
        }
        catch (const std::exception &ex) {
            RCLCPP_WARN(ros_node->get_logger(), "Attempt %d/%d: Error creating tree: %s. Retrying in 1s...", i + 1, max_retries, ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    if (!tree_created) {
        if (!g_shutdown_requested) {
            RCLCPP_ERROR(ros_node->get_logger(), "Failed to create tree after %d attempts. Exiting.", max_retries);
        }
        return 1;
    }

    // Grootで可視化するためのパブリッシャー
    RCLCPP_INFO(ros_node->get_logger(), "Initializing Groot Publisher on ports 2666 (Publisher) and 2667 (Server)");
    BT::PublisherZMQ publisher_zmq(tree, 25, 2666, 2667);

    RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree Started");

    // 5. 実行ループ
    int tick_count = 0; 
    while (rclcpp::ok() && !g_shutdown_requested) {
        if (tick_count % 10 == 0) {
            RCLCPP_INFO(ros_node->get_logger(), "--- Ticking Root (%d) ---", tick_count);
        }
        
        try {
            tree.tickRoot();
        } catch (const std::exception &ex) {
             RCLCPP_ERROR(ros_node->get_logger(), "Error during tick: %s", ex.what());
        }
        
        rclcpp::spin_some(ros_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tick_count++;
    }

    RCLCPP_INFO(ros_node->get_logger(), "Shutting down Behavior Tree Executor...");
    tree.haltTree();
    rclcpp::shutdown();
    return 0;
}