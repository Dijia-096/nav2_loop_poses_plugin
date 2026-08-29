cmake_minimum_required(VERSION 3.8)
project(nav2_loop_poses_plugin)

# 只保留真正需要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)

# 需要 nav2_package() 的话加这个
find_package(nav2_common REQUIRED)
nav2_package()

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2_ros
  nav2_util
)

include_directories(include)

# ★ 编译你的插件共享库
add_library(${PROJECT_NAME} SHARED
  src/nav2_loop_poses_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

# ★ 安装
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# ★ 导出给下游（通常不需要，但保留）
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
