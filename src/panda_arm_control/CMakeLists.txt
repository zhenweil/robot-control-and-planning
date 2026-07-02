cmake_minimum_required(VERSION 3.8)
project(panda_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(VTK 9.1 REQUIRED COMPONENTS
	CommonCore
	CommonDataModel
	CommonTransforms
	FiltersCore
	FiltersGeneral
	FiltersModeling
	IOGeometry
	)

add_executable(panda_arm_control src/panda_arm_control.cpp)

ament_target_dependencies(panda_arm_control
	rclcpp
	moveit_ros_planning_interface
	moveit_msgs
	geometry_msgs
	shape_msgs
	geometric_shapes
	)

add_executable(viewpoint_planner
	src/mesh_utils.cpp
	src/viewpoint_planner_node.cpp
	)

target_include_directories(viewpoint_planner PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/include
	)

ament_target_dependencies(viewpoint_planner
	rclcpp
	moveit_ros_planning_interface
	moveit_ros_planning
	moveit_msgs
	geometry_msgs
	visualization_msgs
	ament_index_cpp
	)

target_link_libraries(viewpoint_planner
	${VTK_LIBRARIES}
	jsoncpp_lib
	)

vtk_module_autoinit(
	TARGETS viewpoint_planner
	MODULES ${VTK_LIBRARIES}
	)

install (DIRECTORY meshes
		DESTINATION share/${PROJECT_NAME}
	)

install(TARGETS panda_arm_control viewpoint_planner
	DESTINATION lib/${PROJECT_NAME}
	)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
