cmake_minimum_required(VERSION 3.8)
project(pnd_adam_u_deploy)
set(CMAKE_CXX_STANDARD 17)
# 编译器设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ========== 库配置 ==========
add_library(${PROJECT_NAME} SHARED
  src/pnd_adam_u_deploy.cpp
  src/FSMstate/fsm_state.cpp
  src/FSMstate/joystick.cpp
  src/FSMstate/robot_handler.cpp
  src/FSMstate/robot_common.cpp
  src/FSMstate/basic_function.cpp

  src/pnd_retarget/retarget.cpp
  src/pnd_retarget/retarget_api.cpp

  src/RobotControl/public_parament.cpp
)

# 添加库的依赖配置
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  Eigen3
)

# 包含目录配置
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/FSMstate>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pnd_retarget>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/RobotControl>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/third_party/json>
  $<INSTALL_INTERFACE:include>
)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


# ========== 可执行文件配置 ==========
add_executable(publisher_node
  src/demo_node.cpp
)

target_include_directories(publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/pndrobotros2/include
)

target_link_directories(publisher_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/pndrobotros2/lib
)

target_link_libraries(publisher_node
  ${PROJECT_NAME}
  fmt::fmt
  yaml-cpp

  pndrobotros2__rosidl_typesupport_cpp
  pndrobotros2__rosidl_generator_c
  pndrobotros2__rosidl_typesupport_fastrtps_cpp
  pndrobotros2__rosidl_typesupport_introspection_cpp
)

ament_target_dependencies(publisher_node
  rclcpp
  ament_index_cpp
)

# ========== 安装目标配置 ==========
# 安装库文件到 lib/
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# 安装可执行文件到 lib/pnd_adam_u_deploy/
install(TARGETS publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 设置RPATH（关键配置）
if(UNIX)
  # 设置安装后的运行时路径
  set_target_properties(publisher_node PROPERTIES
    INSTALL_RPATH "$ORIGIN/.."   # 从 lib/pnd_adam_u_deploy 向上查找
  )
  # 构建时也能正确找到库
  set_target_properties(publisher_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
  )
endif()

# 安装资源目录
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置目录
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pndrobotros2/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# ========== 导出依赖配置 ==========
ament_export_dependencies(
  rclcpp
)

# 最终配置
ament_package()
