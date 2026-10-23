cmake_minimum_required(VERSION 3.16.3)
project(flexiv_drdk-examples)

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Minimum example list for all OS
set(EXAMPLE_LIST
  basics1_display_robot_states
  basics2_clear_fault
  basics3_self_collision_avoidance
  basics4_primitive_execution
  basics5_plan_execution
  basics6_zero_force_torque_sensors
  basics7_gripper_control
  basics8_auto_recovery
  basics9_update_robot_tool
  basics10_global_variables
  intermediate1_realtime_joint_position_control
  intermediate2_realtime_joint_impedance_control
  intermediate3_realtime_joint_torque_control
  intermediate4_realtime_joint_floating
  intermediate5_realtime_cartesian_pure_motion_control
  intermediate6_realtime_cartesian_motion_force_control
  intermediate7_robot_dynamics
  intermediate8_bimanual_tensioned_motion
)

# Find flexiv_drdk INTERFACE library
find_package(flexiv_drdk REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_drdk)

  # C++17 required
  set_target_properties(${example} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
endforeach()
