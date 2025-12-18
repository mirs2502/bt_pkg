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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

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
            // "_bt_node.so" で終わるファイル（Nav2のBTノード）を探す
            if (filename.find("_bt_node.so") != std::string::npos) {
                // Waitノードは自作モックを使うので、Nav2のプラグインは除外する
                if (filename.find("wait") != std::string::npos) {
                    continue;
                }
                // 例: libnav2_wait_action_bt_node.so などが見つかる
                factory.registerFromPlugin(entry.path().string());
                // RCLCPP_INFO(ros_node->get_logger(), "Loaded plugin: %s", filename.c_str());
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(ros_node->get_logger(), "Failed to load plugins: %s", e.what());
    }
    

    factory.registerNodeType<HelloRos>("HelloRos");
    factory.registerNodeType<PublishTest>("PublishTest");

    // 3. Blackboardの設定とROSノードの登録
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", ros_node); // これで全ノードがROS機能を使えるようになる
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10)); // Nav2ノード用
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000)); // Nav2ノード用
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000)); // Nav2ノード用

    // 4. ツリーの構築
    try {
        auto tree = factory.createTreeFromFile(bt_xml_path, blackboard);

        // Grootで可視化するためのパブリッシャー
        // Nav2のbehavior_serverとポート(1666/1667)が競合するため、1668/1669に変更
        BT::PublisherZMQ publisher_zmq(tree, 25, 1668, 1669);

        RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree Started");

        // 5. 実行ループ (TickとSpinの共存)
        //BT::NodeStatus status = BT::NodeStatus::RUNNING;
        // 実行中(RUNNING)である限りループする（成功したら終わる）
        //while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        //    status = tree.tickRoot();
        //    rclcpp::spin_some(ros_node);
        //    
            // CPU負荷を下げるためのスリープ（適宜調整）
        //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //}
        
        // 5. 実行ループ
        int tick_count = 0; // カウンタ追加
        while (rclcpp::ok()) {
            // 動作確認のため、10回に1回（約1秒ごとに）ログを出す
            if (tick_count % 10 == 0) {
                RCLCPP_INFO(ros_node->get_logger(), "--- Ticking Root (%d) ---", tick_count);
            }
            
            tree.tickRoot(); // ツリーの実行
            
            rclcpp::spin_some(ros_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            tick_count++;
        }

    }
    catch (const std::exception &ex) {
        RCLCPP_ERROR(ros_node->get_logger(), "Error creating tree: %s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
}