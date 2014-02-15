////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_control
//
// File:      pan_tilt_control.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS pan_tilt_control node class implementation.
 *
 * \author Danial Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"


//
// ROS generated core, industrial, and pan-tilt messages.
//
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "pan_tilt_control/JointStateExtended.h"
#include "pan_tilt_control/RobotStatusExtended.h"

//
// ROS generatated pan-tilt services.
//
#include "pan_tilt_control/ClearAlarmsSvc.h"
#include "pan_tilt_control/EStopSvc.h"
#include "pan_tilt_control/FreezeSvc.h"
#include "pan_tilt_control/GetProductInfoSvc.h"
#include "pan_tilt_control/GotoZeroPtSvc.h"
#include "pan_tilt_control/IsAlarmedSvc.h"
#include "pan_tilt_control/IsCalibratedSvc.h"
#include "pan_tilt_control/ReleaseSvc.h"
#include "pan_tilt_control/ResetEStopSvc.h"
#include "pan_tilt_control/SetRobotModeSvc.h"
#include "pan_tilt_control/StopSvc.h"

//
// ROS generated action servers.
//
#include "pan_tilt_control/CalibrateAction.h"

//
// RoadNarrows embedded pan-tilt library.
//
#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptRobot.h"

//
// Node headers.
//
#include "pan_tilt_control.h"


using namespace std;
using namespace pan_tilt;
using namespace pan_tilt_control;


//------------------------------------------------------------------------------
// PanTiltControl Class
//------------------------------------------------------------------------------

PanTiltControl::PanTiltControl(ros::NodeHandle &nh) : m_nh(nh)
{
}

PanTiltControl::~PanTiltControl()
{
  disconnect();
}


//..............................................................................
// Services
//..............................................................................

void PanTiltControl::advertiseServices()
{
  string  strSvc;

  strSvc = "clear_alarms";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::clearAlarms,
                                          &(*this));

  strSvc = "estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::estop,
                                          &(*this));

  strSvc = "freeze";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::freeze,
                                          &(*this));

  strSvc = "get_product_info";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::getProductInfo,
                                          &(*this));

  strSvc = "goto_zero";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::gotoZeroPt,
                                          &(*this));

  strSvc = "is_alarmed";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::isAlarmed,
                                          &(*this));

  strSvc = "is_calibrated";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::isCalibrated,
                                          &(*this));

  strSvc = "release";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::release,
                                          &(*this));

  strSvc = "reset_estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::resetEStop,
                                          &(*this));

  strSvc = "set_robot_mode";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::setRobotMode,
                                          &(*this));

  strSvc = "stop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &PanTiltControl::stop,
                                          &(*this));
}

bool PanTiltControl::clearAlarms(ClearAlarmsSvc::Request  &req,
                                 ClearAlarmsSvc::Response &rsp)
{
  ROS_DEBUG("clear_alarms");

  m_robot.clearAlarms();

  return true;
}

bool PanTiltControl::estop(EStopSvc::Request  &req,
                           EStopSvc::Response &rsp)
{
  ROS_DEBUG("estop");

  m_robot.estop();

  ROS_INFO("ESTOPPED! You must issue a \"reset_estop\" to continue.");

  return true;
}

bool PanTiltControl::freeze(FreezeSvc::Request  &req,
                            FreezeSvc::Response &rsp)
{
  ROS_DEBUG("freeze");

  m_robot.freeze();

  return true;
}

bool PanTiltControl::getProductInfo(GetProductInfoSvc::Request  &req,
                                    GetProductInfoSvc::Response &rsp)
{
  int   nMajor, nMinor, nRev;

  ROS_DEBUG("get_product_info");

  m_robot.getVersion(nMajor, nMinor, nRev);

  rsp.i.maj             = nMajor;
  rsp.i.min             = nMinor;
  rsp.i.rev             = nRev;
  rsp.i.version_string  = m_robot.getVersion();
  rsp.i.product_id      = m_robot.getProdId();
  rsp.i.product_name    = m_robot.getProdName();
  rsp.i.desc            = m_robot.getFullProdBrief();

  return true;
}

bool PanTiltControl::gotoZeroPt(GotoZeroPtSvc::Request  &req,
                                GotoZeroPtSvc::Response &rsp)
{
  ROS_DEBUG("goto_zero");

  m_robot.gotoZeroPtPos();

  ROS_INFO("Moving pan-tilt to calibrated zero point.");

  return true;
}

bool PanTiltControl::isAlarmed(IsAlarmedSvc::Request  &req,
                               IsAlarmedSvc::Response &rsp)
{
  ROS_DEBUG("is_alarmed");

  rsp.is_alarmed = m_robot.isAlarmed();

  return true;
}

bool PanTiltControl::isCalibrated(IsCalibratedSvc::Request  &req,
                                  IsCalibratedSvc::Response &rsp)
{
  ROS_DEBUG("is_calibrated");

  rsp.is_calibrated = m_robot.isCalibrated();

  return true;
}

bool PanTiltControl::release(ReleaseSvc::Request  &req,
                             ReleaseSvc::Response &rsp)
{
  ROS_DEBUG("release");

  m_robot.release();

  return true;
}

bool PanTiltControl::resetEStop(ResetEStopSvc::Request  &req,
                                ResetEStopSvc::Response &rsp)
{
  ROS_DEBUG("reset_estop");

  m_robot.resetEStop();

  ROS_INFO("EStop reset.");

  return true;
}

bool PanTiltControl::setRobotMode(SetRobotModeSvc::Request  &req,
                                  SetRobotModeSvc::Response &rsp)
{
  ROS_DEBUG("set_robot_mode");

  m_robot.setRobotMode((PanTiltRobotMode)req.mode.val);

  return true;
}

bool PanTiltControl::stop(StopSvc::Request  &req,
                          StopSvc::Response &rsp)
{
  ROS_DEBUG("stop");

  // future m_robot.stop();

  return true;
}


//..............................................................................
// Topic Publishers
//..............................................................................

void PanTiltControl::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "joint_states";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::JointState>(strPub, nQueueDepth);

  strPub = "joint_states_ex";
  m_publishers[strPub] =
    m_nh.advertise<pan_tilt_control::JointStateExtended>(strPub,nQueueDepth);

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);

  strPub = "robot_status_ex";
  m_publishers[strPub] =
    m_nh.advertise<pan_tilt_control::RobotStatusExtended>(strPub,nQueueDepth);
}

void PanTiltControl::publish()
{
  publishJointState();
  publishRobotStatus();
}

void PanTiltControl::publishJointState()
{
  PanTiltJointStatePoint    state;

  // get robot's extended joint state.
  m_robot.getJointState(state);
  
  // update joint state message
  updateJointStateMsg(state, m_msgJointState);

  // publish joint state messages
  m_publishers["joint_states"].publish(m_msgJointState);

  // update extended joint state message
  updateExtendedJointStateMsg(state, m_msgJointStateEx);

  // publish extened joint state messages
  m_publishers["joint_states_ex"].publish(m_msgJointStateEx);
}

void PanTiltControl::publishRobotStatus()
{
  PanTiltRobotStatus  status;

  // get robot's extended status.
  m_robot.getRobotStatus(status);

  // update robot status message
  updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  m_publishers["robot_status"].publish(m_msgRobotStatus);

  // update extended robot status message
  updateExtendedRobotStatusMsg(status, m_msgRobotStatusEx);

  // publish extened robot status message
  m_publishers["robot_status_ex"].publish(m_msgRobotStatusEx);
}

void PanTiltControl::updateJointStateMsg(PanTiltJointStatePoint &state,
                                         sensor_msgs::JointState &msg)
{
  //
  // Clear previous joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  //
  // Set joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set joint state state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    // joint state
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
  }
}

void PanTiltControl::updateExtendedJointStateMsg(PanTiltJointStatePoint &state,
                                     pan_tilt_control::JointStateExtended &msg)
{
  pan_tilt_control::OpState opstate;

  // 
  // Clear previous extended joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();
  msg.master_servo_id.clear();
  msg.slave_servo_id.clear();
  msg.odometer_pos.clear();
  msg.encoder_pos.clear();
  msg.raw_speed.clear();
  msg.op_state.clear();

  //
  // Set extended joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set extended joint state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
    msg.master_servo_id.push_back(state[n].m_nMasterServoId);
    msg.slave_servo_id.push_back(state[n].m_nSlaveServoId);
    msg.odometer_pos.push_back(state[n].m_nOdPos);
    msg.encoder_pos.push_back(state[n].m_nEncPos);
    msg.raw_speed.push_back(state[n].m_nSpeed);

    opstate.calib_state = state[n].m_eOpState;
    msg.op_state.push_back(opstate);
  }
}

void PanTiltControl::updateRobotStatusMsg(PanTiltRobotStatus &status,
                                          industrial_msgs::RobotStatus &msg)
{
  //
  // Set robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set industrial message compliant robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;

}

void PanTiltControl::updateExtendedRobotStatusMsg(
                                    PanTiltRobotStatus &status,
                                    pan_tilt_control::RobotStatusExtended &msg)
{
  ServoHealth sh;
  int         i;

  //
  // Set extended robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set pan-tilt message extended robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;
  msg.is_calibrated.val   = status.m_eIsCalibrated;

  // clear previous data
  msg.servo_health.clear();

  for(i=0; i<status.m_vecServoHealth.size(); ++i)
  {
    sh.servo_id = status.m_vecServoHealth[i].m_nServoId;
    sh.temp     = status.m_vecServoHealth[i].m_fTemperature;
    sh.voltage  = status.m_vecServoHealth[i].m_fVoltage;
    sh.alarm    = status.m_vecServoHealth[i].m_uAlarms;

    msg.servo_health.push_back(sh);
  }
}


//..............................................................................
// Subscribed Topics
//..............................................................................

void PanTiltControl::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "joint_command";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &PanTiltControl::execJointCmd,
                                          &(*this));
}

void PanTiltControl::execJointCmd(const trajectory_msgs::JointTrajectory &jt)
{
  ROS_DEBUG("Executing joint_command.");

  PanTiltJointTrajectoryPoint pt;

  // load trajectory point
  for(int j=0; j<jt.joint_names.size(); ++j)
  {
    pt.append(jt.joint_names[j],
              jt.points[0].positions[j], 
              jt.points[0].velocities[j]);
    ROS_INFO("j = %d pos=%2.1f speed=%2.1f", j, 
                                            jt.points[0].positions[j], 
                                            jt.points[0].velocities[j]);
  }

  m_robot.move(pt);
}
