cmake_minimum_required(VERSION 3.8)
project(panda_arm_control)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

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
	moveit_ros_planning
	moveit_msgs
	geometry_msgs
	shape_msgs
	geometric_shapes
	visualization_msgs
	)

add_executable(viewpoint_planner_rkga
	src/mesh_utils.cpp
	src/potential_field_sampler.cpp
	src/real_cost_planning.cpp
	src/rkga_scp.cpp
	src/viewpoint_io.cpp
	src/viewpoint_planner_rkga_node.cpp
	)

add_executable(viewpoint_planner_nn2opt
	src/mesh_utils.cpp
	src/potential_field_sampler.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/viewpoint_planner_nn2opt_node.cpp
	)

add_executable(viewpoint_planner_hgtsp
	src/mesh_utils.cpp
	src/potential_field_sampler.cpp
	src/real_cost_planning.cpp
	src/hierarchical_tour.cpp
	src/viewpoint_io.cpp
	src/viewpoint_planner_hgtsp_node.cpp
	)

add_executable(base_placement
	src/base_placement.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/base_placement_node.cpp
	)

add_executable(base_gradient
	src/base_gradient.cpp
	src/base_placement.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/base_gradient_node.cpp
	)

foreach(viewpoint_planner_target viewpoint_planner_rkga viewpoint_planner_nn2opt viewpoint_planner_hgtsp base_placement base_gradient)
	target_include_directories(${viewpoint_planner_target} PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

	ament_target_dependencies(${viewpoint_planner_target}
		rclcpp
		moveit_ros_planning_interface
		moveit_ros_planning
		moveit_msgs
		geometry_msgs
		shape_msgs
		geometric_shapes
		visualization_msgs
		ament_index_cpp
		)

	target_link_libraries(${viewpoint_planner_target}
		${VTK_LIBRARIES}
		jsoncpp_lib
		)

	vtk_module_autoinit(
		TARGETS ${viewpoint_planner_target}
		MODULES ${VTK_LIBRARIES}
		)
endforeach()

install (DIRECTORY meshes config
		DESTINATION share/${PROJECT_NAME}
	)

install(TARGETS panda_arm_control viewpoint_planner_rkga viewpoint_planner_nn2opt viewpoint_planner_hgtsp base_placement base_gradient
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
