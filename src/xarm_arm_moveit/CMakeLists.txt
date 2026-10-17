cmake_minimum_required(VERSION 3.8)
project(xarm_arm_moveit)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY config meshes urdf
		DESTINATION share/${PROJECT_NAME}
		)

ament_package()
