cmake_minimum_required(VERSION 3.8)
project(robot_topological_explore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_PREFIX_PATH "/home/depressedcoder/eigen-3.4.0;/home/depressedcoder/BehaviorTree.CPP;")
set(CMAKE_CXX_STANDARD 17)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(robot_topological_explore src/topological_explore_node.cpp src/costmap_client.cpp)
ament_target_dependencies(robot_topological_explore rclcpp nav_msgs nav2_msgs rclcpp_action tf2_ros nav2_costmap_2d tf2_geometry_msgs map_msgs visualization_msgs std_msgs builtin_interfaces behaviortree_cpp_v3)
target_link_libraries(robot_topological_explore Eigen3::Eigen)


target_include_directories(robot_topological_explore PUBLIC  
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_topological_explore PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS robot_topological_explore
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
