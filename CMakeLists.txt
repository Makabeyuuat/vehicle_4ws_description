cmake_minimum_required(VERSION 3.0.2)
# CMP0048: project()でVERSIONを管理するポリシーをNEWに設定
cmake_policy(SET CMP0048 NEW)
project(vehicle_4ws_description VERSION 0.0.1)

## C++11を有効化
add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")

## catkinのマクロとライブラリを検索
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs  # 正しいパッケージ名
)

## パッケージを宣言
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf2_ros tf2_geometry_msgs
)

## ヘッダ検索パスを追加
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## 実行ファイルの定義
add_executable(test_motion_controller
  src/test_motion.cpp
  src/vehicle.cpp
  src/callback.cpp
)
target_link_libraries(test_motion_controller
  ${catkin_LIBRARIES}
)

## 実行ファイルの定義
add_executable(Xaxis_tracking_controller
  src/Xaxis_tracking.cpp
  src/vehicle.cpp
  src/callback.cpp
  src/Bezier.cpp
  src/differential_equations.cpp
  src/initial.cpp
  src/mathFunc.cpp
  src/getInputValue.cpp
)
target_link_libraries(Xaxis_tracking_controller
  ${catkin_LIBRARIES}
)

install(TARGETS Xaxis_tracking_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## 実行ファイルの定義
add_executable(vehicle_dynamics_controller
  src/vehicle_dynamics.cpp
  src/vehicle.cpp
  src/callback.cpp
  src/Bezier.cpp
  src/differential_equations_dynamics.cpp
  src/initial.cpp
  src/mathFunc.cpp
  src/getInputValue_dynamics.cpp
  src/dynamics_integrator.cpp
)
target_link_libraries(vehicle_dynamics_controller
  ${catkin_LIBRARIES}
)



add_executable(trajectory_path_publisher 
  src/trajectory_path_publisher.cpp
  src/mathFunc.cpp         # Rx, Ry を実装しているファイル
  src/Bezier.cpp           # 必要なら
  src/initial.cpp          # 必要なら
  )
target_link_libraries(trajectory_path_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_path_publisher ${catkin_EXPORTED_TARGETS})
#### 追加ノードがあればここに追記 ####
# add_executable(steering_desired_controller src/steering_desired_controller.cpp src/vehicle.cpp)
# target_link_libraries(steering_desired_controller ${catkin_LIBRARIES})
