cmake_minimum_required(VERSION 2.8.3)
project(libpan_tilt)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr-pantilt_DIR /usr/local/lib/cmake/rnr)
find_package(rnr-pantilt
  REQUIRED
)

#message(STATUS, "${rnr-pantilt_LIBRARIES}")

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
  ../include
  ${rnr-pantilt_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(RN_LOGGING_COMPILE_FLAGS "-DLOG -DLOGMOD=\\\"libpan_tilt\\\"")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${RN_LOGGING_COMPILE_FLAGS}")
#message(STATUS, ${CMAKE_CXX_FLAGS})

add_library(pan_tilt
  src/ptCalib.cxx
  src/ptDesc.cxx
  src/ptJoint.cxx
  src/ptProdAX.cxx
  src/ptProdMX.cxx
  src/ptRobot.cxx
  src/ptSpec.cxx
  src/ptStatus.cxx
  src/ptUtils.cxx
)

target_link_libraries(pan_tilt 
  ${rnr-pantilt_LIBRARIES}
)

install(TARGETS pan_tilt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ../include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
