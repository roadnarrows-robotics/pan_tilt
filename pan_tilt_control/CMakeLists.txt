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

set(rnr_DIR /prj/lib/rnr)
find_package(rnr)

#message(STATUS, "${rnr_LIBRARIES}")

#catkin_python_setup()

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
    CalibrateSvc.srv 
    ClearAlarmsSvc.srv 
    EStopSvc.srv 
    FreezeSvc.srv 
    GetProductInfoSvc.srv 
    GotoZeroPtSvc.srv 
    IsAlarmedSvc.srv 
    IsCalibratedSvc.srv 
    ReleaseSvc.srv
    ResetEStopSvc.srv 
    SetRobotModeSvc.srv
    StopSvc.srv
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

add_dependencies(pan_tilt_control pan_tilt_control_gencpp)

#install(
#  PROGRAMS 
#    scripts/pantilt_panel
#    scripts/pantilt_teleop
#    scripts/joint_trajectory_demo 
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
