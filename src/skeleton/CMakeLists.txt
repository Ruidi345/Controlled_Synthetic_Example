cmake_minimum_required(VERSION 3.8)
project(skeleton)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==== 新增：ComposableNode 组件需要 rclcpp_components ====
find_package(rclcpp_components REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  "msg/Circle.msg"
  "msg/Square.msg"
  "msg/Num.msg"
  "srv/TwoNumbers.srv"
  "srv/OddOrEven.srv"
)

# ==== 修改：把 rclcpp_components 也导出（可选但推荐）====
ament_export_dependencies(rosidl_default_runtime rclcpp rclcpp_components)

# ------------------------------------------------------------------------------
# 原来的可执行节点
# ------------------------------------------------------------------------------

# ==== 修改：SquareArea 和 CircleGenerator 改为组件，不再编译为独立可执行 ====
# add_executable(square_area src/square_area.cpp)
# add_executable(circle_generator src/circle_generator.cpp)

add_executable(square_generator src/square_generator.cpp)
add_executable(perimeter_node src/perimeter_node.cpp)
add_executable(circle_area src/circle_area.cpp)

# 这里不再对 square_area / circle_generator 做可执行的依赖设置
# ament_target_dependencies(square_area rclcpp std_msgs)
# ament_target_dependencies(circle_generator rclcpp std_msgs)
ament_target_dependencies(square_generator rclcpp std_msgs)
ament_target_dependencies(perimeter_node rclcpp std_msgs)
ament_target_dependencies(circle_area rclcpp std_msgs)

# add_dependencies(square_area ${PROJECT_NAME}__rosidl_generator_cpp)
# rosidl_target_interfaces(square_area ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_dependencies(circle_generator ${PROJECT_NAME}__rosidl_generator_cpp)
# rosidl_target_interfaces(circle_generator ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_dependencies(square_generator ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(square_generator ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_dependencies(perimeter_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(perimeter_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_dependencies(circle_area ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(circle_area ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ------------------------------------------------------------------------------
# 新增：组件库（SquareArea + CircleGenerator 作为 ComposableNode 组件）
# ------------------------------------------------------------------------------

# ==== 新增：把没有 main() 的 SquareArea / CircleGenerator 编译成一个共享库 ====
add_library(${PROJECT_NAME}_components SHARED
  src/square_area.cpp
  src/circle_generator.cpp
)

# ==== 新增：组件库依赖 rclcpp / rclcpp_components / std_msgs ====
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  std_msgs
)

# ==== 新增：组件库也要依赖本包生成的接口（自定义 msg/srv）====
add_dependencies(${PROJECT_NAME}_components ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(${PROJECT_NAME}_components ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==== 新增：注册组件类名（和你 cpp 里 RCLCPP_COMPONENTS_REGISTER_NODE(...) 里的类名对应）====
# 假设你的类名是：class SquareArea : public rclcpp::Node {...}
# 和        class CircleGenerator : public rclcpp::Node {...}
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "SquareArea"
  "CircleGenerator"
)

# ament_python_install_package(${PROJECT_NAME})

# 可执行文件（给 <node exec="..."> 用），还是放在 lib/${PROJECT_NAME}
install(TARGETS
  square_generator
  perimeter_node
  circle_area
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ 组件库（给 ComposableNode 用），必须放在 lib/ 根目录
install(TARGETS
  ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin  # 一般不会用到，但写上也无妨
)

install(PROGRAMS
  scripts/age_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})

ament_package()
