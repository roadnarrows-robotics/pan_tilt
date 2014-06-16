cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  industrial_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr_DIR /usr/local/lib/cmake/rnr)
find_package(rnr)

#message(STATUS, "${rnr_LIBRARIES}")

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    OpState.msg 
    ProductInfo.msg 
    JointStateExtended.msg 
    RobotStatusExtended.msg
    ServoHealth.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    Calibrate.srv 
    ClearAlarms.srv 
    EStop.srv 
    Freeze.srv 
    GetProductInfo.srv 
    GotoZeroPt.srv 
    IsAlarmed.srv 
    IsCalibrated.srv 
    Pan.srv
    Release.srv
    ResetEStop.srv 
    SetRobotMode.srv
    Stop.srv
    Sweep.srv
)

add_action_files(
  DIRECTORY action 
  FILES 
    Calibrate.action
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  industrial_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

add_executable(pan_tilt_control
  src/pan_tilt_as_calib.cpp
  src/pan_tilt_control.cpp
  src/pan_tilt_control_main.cpp
)

add_executable(pan_tilt_teleop
  src/pan_tilt_teleop/pan_tilt_teleop.cpp
  src/pan_tilt_teleop/pan_tilt_teleop_main.cpp
)

include_directories(
  include 
  ../include
  ${rnr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(pan_tilt_control 
  pan_tilt
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(pan_tilt_teleop 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(pan_tilt_control pan_tilt_control_gencpp)
add_dependencies(pan_tilt_teleop pan_tilt_control_gencpp)

install(
  PROGRAMS 
    scripts/pan_tilt_panel
#    scripts/pan_tilt_teleop
#    scripts/pan_tilt_trajectory_demo 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
