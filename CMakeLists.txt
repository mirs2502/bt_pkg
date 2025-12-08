cmake_minimum_required(VERSION 3.8)
project(bt_pkg) # パッケージ名をbt_pkgに統一

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED) 
find_package(mirs_msgs REQUIRED) 

# 自分のパッケージ内のヘッダー置き場を指定
include_directories(
  include
)

# 実行ファイルの定義
add_executable(bt_executor
  src/main.cpp
)

# 【重要】ここを修正しました
# ament_target_dependenciesを使うと、ヘッダーのパス(include)と
# ライブラリのリンク(lib)を両方自動で設定してくれます。
ament_target_dependencies(bt_executor
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  std_msgs
  nav_msgs
  nav2_msgs
  nav2_behavior_tree
  mirs_msgs
)

# インストール設定（実行ファイル）
install(TARGETS bt_executor
  DESTINATION lib/${PROJECT_NAME})

# インストール設定（ディレクトリ）
install(DIRECTORY
  behavior_tree
  launch
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
