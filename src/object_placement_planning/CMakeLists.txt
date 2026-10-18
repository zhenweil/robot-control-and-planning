cmake_minimum_required(VERSION 3.8)
project(object_placement_planning)

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
find_package(random_numbers REQUIRED)
find_package(ompl REQUIRED)
find_package(jsoncpp REQUIRED)
# osqp_vendor (ros-humble-osqp-vendor) makes upstream OSQP's own CMake package findable.
find_package(osqp REQUIRED)
find_package(VTK 9.1 REQUIRED COMPONENTS
	CommonCore
	CommonDataModel
	CommonTransforms
	FiltersCore
	FiltersGeneral
	FiltersModeling
	IOGeometry
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

add_executable(bstar_placement
	src/bstar_placement.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/bstar_placement_node.cpp
	)

add_executable(object_placement
	src/object_placement.cpp
	src/closest_ik.cpp
	src/gtsp.cpp
	src/ik_sampling.cpp
	src/joint_travel.cpp
	src/manipulability.cpp
	src/marker_utils.cpp
	src/object_placement_optimizer.cpp
	src/room_margins.cpp
	src/scene_geometry.cpp
	src/tour_placement.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/object_placement_node.cpp
	)

# Builds the reachability map that random_placement's weighted sampling loads.
add_executable(build_reachability_map
	src/reachability_map.cpp
	src/build_reachability_map.cpp
	)
target_include_directories(build_reachability_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(build_reachability_map rclcpp moveit_ros_planning random_numbers)

# Viewpoints with no robot reachability check, for object_placement's tour_input_dir.
add_executable(viewpoint_generator
	src/mesh_utils.cpp
	src/potential_field_sampler.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/viewpoint_generation.cpp
	src/viewpoint_generator_node.cpp
	)

# Random-sampling placement baseline (separate from object_placement).
add_executable(random_placement
	src/gtsp.cpp
	src/ik_sampling.cpp
	src/joint_travel.cpp
	src/reachability_map.cpp
	src/real_cost_planning.cpp
	src/viewpoint_io.cpp
	src/random_placement_node.cpp
	)

# SGD placement baseline (Mayer and Althoff 2026), separate from object_placement.
add_executable(sgd_placement
	src/closest_ik.cpp
	src/gtsp.cpp
	src/joint_travel.cpp
	src/manipulability.cpp
	src/real_cost_planning.cpp
	src/scene_geometry.cpp
	src/viewpoint_io.cpp
	src/sgd_placement_node.cpp
	)

# One-off check: is a missed viewpoint reachable at a given object position (heavy IK search).
add_executable(ik_reachability_check
	src/closest_ik.cpp
	src/manipulability.cpp
	src/real_cost_planning.cpp
	src/scene_geometry.cpp
	src/viewpoint_io.cpp
	src/ik_reachability_check.cpp
	)

foreach(viewpoint_planner_target viewpoint_planner_nn2opt viewpoint_planner_hgtsp bstar_placement object_placement random_placement sgd_placement ik_reachability_check viewpoint_generator)
	target_include_directories(${viewpoint_planner_target} PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

	ament_target_dependencies(${viewpoint_planner_target}
		rclcpp
		moveit_ros_planning_interface
		moveit_ros_planning
		moveit_msgs
		random_numbers
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

# bstar_placement's B* implementation solves a sequence of linear programs with OSQP.
target_link_libraries(bstar_placement osqp::osqp)

# object_placement seeds OMPL (planned travel) from RANDOM_SEED so runs repeat.
target_link_libraries(object_placement ompl::ompl osqp::osqp)

install (DIRECTORY meshes config
		DESTINATION share/${PROJECT_NAME}
	)

install(TARGETS viewpoint_planner_nn2opt viewpoint_planner_hgtsp bstar_placement object_placement random_placement sgd_placement ik_reachability_check build_reachability_map viewpoint_generator
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
