////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_teleop
//
// File:      pan_tilt_teleop.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS pan_tilt_teleop node class implementation.
 *
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

//
// System
//
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <string>
#include <map>

//
// Boost
//
#include <boost/bind.hpp>
#include "boost/assign.hpp"

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

//
// ROS generated core, industrial, and pan_tilt messages.
//
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "pan_tilt_control/JointStateExtended.h"
#include "pan_tilt_control/RobotStatusExtended.h"

//
// ROS generatated pan_tilt services.
//
#include "pan_tilt_control/Calibrate.h"
#include "pan_tilt_control/ClearAlarms.h"
#include "pan_tilt_control/EStop.h"
#include "pan_tilt_control/Freeze.h"
#include "pan_tilt_control/GetProductInfo.h"
#include "pan_tilt_control/GotoZeroPt.h"
#include "pan_tilt_control/IsAlarmed.h"
#include "pan_tilt_control/IsCalibrated.h"
#include "pan_tilt_control/Pan.h"
#include "pan_tilt_control/Release.h"
#include "pan_tilt_control/ResetEStop.h"
#include "pan_tilt_control/SetRobotMode.h"
#include "pan_tilt_control/Stop.h"
#include "pan_tilt_control/Sweep.h"

//
// ROS generated action servers.
//

//
// ROS generated HID messages.
//
#include "hid/ConnStatus.h"           // subscribe
#include "hid/Controller360State.h"   // subscribe
#include "hid/LEDPattern.h"           // service
#include "hid/RumbleCmd.h"            // publish

//
// ROS generatated HID services.
//
#include "hid/SetLED.h"
#include "hid/SetRumble.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/hid/HIDXbox360.h"

//
// RoadNarrows embedded pan_tilt library.
//
#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"
#include "pan_tilt/ptTraj.h"
#include "pan_tilt/ptRobot.h"

//
// Node headers.
//
#include "pan_tilt_teleop.h"

using namespace std;
using namespace boost::assign;
using namespace hid;
using namespace pan_tilt;
using namespace trajectory_msgs;
using namespace industrial_msgs;
using namespace pan_tilt_control;


//------------------------------------------------------------------------------
// PanTiltTeleop Class
//------------------------------------------------------------------------------

PanTiltTeleop::PanTiltTeleop(ros::NodeHandle &nh, double hz) :
      m_nh(nh), m_hz(hz)
{
  m_eState            = TeleopStateUninit;
  m_bHasXboxComm      = false;
  m_nWdXboxCounter    = 0;
  m_nWdXboxTimeout    = countsPerSecond(3.0);
  m_bRcvdRobotStatus  = false;
  m_bRcvdJointState   = false;
  m_bHasRobotComm     = false;
  m_nWdRobotCounter   = 0;
  m_nWdRobotTimeout   = countsPerSecond(5.0);
  m_bHasFullComm      = false;

  m_buttonState = map_list_of
      (ButtonIdContSweep,   0)
      (ButtonIdEStop,       0)
      (ButtonIdContPan,     0)
      (ButtonIdGotoZeroPt,  0)

      (ButtonIdPause,       0)
      (ButtonIdToggleMode,  0)
      (ButtonIdStart,       0)

      (ButtonIdTilt,        0)
      (ButtonIdPan,         0);

  m_rumbleLeft    = 0;
  m_rumbleRight   = 0;
}

PanTiltTeleop::~PanTiltTeleop()
{
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Server Services
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Client Services
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void PanTiltTeleop::clientServices()
{
  string  strSvc;

  strSvc = "/xbox_360/set_led";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetLED>(strSvc);

  strSvc = "/xbox_360/set_rumble";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetRumble>(strSvc);

  strSvc = "/pan_tilt_control/estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::EStop>(strSvc);

  strSvc = "/pan_tilt_control/freeze";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::Freeze>(strSvc);

  strSvc = "/pan_tilt_control/goto_zero";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::GotoZeroPt>(strSvc);

  strSvc = "/pan_tilt_control/pan";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::Pan>(strSvc);

  strSvc = "/pan_tilt_control/reset_estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::ResetEStop>(strSvc);

  strSvc = "/pan_tilt_control/set_robot_mode";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::SetRobotMode>(strSvc);

  strSvc = "/pan_tilt_control/sweep";
  m_clientServices[strSvc] =
      m_nh.serviceClient<pan_tilt_control::Sweep>(strSvc);
}

void PanTiltTeleop::setLED(int pattern)
{
  hid::SetLED svc;

  svc.request.led_pattern.val = pattern;

  if( m_clientServices["/xbox_360/set_led"].call(svc) )
  {
    ROS_DEBUG("Xbox360 LED set to pattern to %d.", pattern);
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 LED.");
  }
}

void PanTiltTeleop::setRumble(int motorLeft, int motorRight)
{
  hid::SetRumble svc;

  // no change
  if( (motorLeft == m_rumbleLeft) && (motorRight == m_rumbleRight) )
  {
    return;
  }

  svc.request.left_rumble  = motorLeft;
  svc.request.right_rumble = motorRight;

  if( m_clientServices["/xbox_360/set_rumble"].call(svc) )
  {
    ROS_DEBUG("Xbox360 rumble motors set to %d, %d.", motorLeft, motorRight);
    m_rumbleLeft  = motorLeft;
    m_rumbleRight = motorRight;
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 rumble motors.");
  }
}

void PanTiltTeleop::estop()
{
  pan_tilt_control::EStop svc;

  if( m_clientServices["/pan_tilt_control/estop"].call(svc) )
  {
    ROS_INFO("Robot emergency stopped.");
  }
  else
  {
    ROS_ERROR("Failed to estop robot.");
  }
}

void PanTiltTeleop::freeze()
{
  pan_tilt_control::Freeze svc;

  if( m_clientServices["/pan_tilt_control/freeze"].call(svc) )
  {
    ROS_INFO("Robot stopped.");
  }
  else
  {
    ROS_ERROR("Failed to freeze robot.");
  }
}

void PanTiltTeleop::gotoZeroPt()
{
  pan_tilt_control::GotoZeroPt svc;

  if( m_clientServices["/pan_tilt_control/goto_zero"].call(svc) )
  {
    ROS_INFO("Moving to zero point.");
  }
  else
  {
    ROS_ERROR("Failed to move.");
  }
}

void PanTiltTeleop::pan()
{
  pan_tilt_control::Pan svc;

  // TODO maybe read pan_tilt_panel xml to get values?
  svc.request.min_pos   = degToRad(-125.0);
  svc.request.max_pos   = degToRad(125.0);
  svc.request.velocity  = 20.0;

  if( m_clientServices["/pan_tilt_control/pan"].call(svc) )
  {
    ROS_INFO("Continuous panning.");
  }
  else
  {
    ROS_ERROR("Failed to pan.");
  }
}

void PanTiltTeleop::resetEStop()
{
  pan_tilt_control::ResetEStop svc;

  if( m_clientServices["/pan_tilt_control/reset_estop"].call(svc) )
  {
    ROS_INFO("Robot emergency stopped has been reset.");
  }
  else
  {
    ROS_ERROR("Failed to reset estop.");
  }
}

void PanTiltTeleop::setRobotMode(PanTiltRobotMode mode)
{
  pan_tilt_control::SetRobotMode svc;

  svc.request.mode.val = mode;

  if( m_clientServices["/pan_tilt_control/set_robot_mode"].call(svc) )
  {
    ROS_INFO("Robot mode set to %d.", mode);
  }
  else
  {
    ROS_ERROR("Failed to set robot mode.");
  }
}

void PanTiltTeleop::sweep()
{
  pan_tilt_control::Sweep svc;

  // TODO maybe read pan_tilt_panel xml to get values?
  svc.request.pan_min_pos   = degToRad(-125.0);
  svc.request.pan_max_pos   = degToRad(125.0);
  svc.request.pan_velocity  = 10.0;
  svc.request.tilt_min_pos  = degToRad(10.0);
  svc.request.tilt_max_pos  = degToRad(90.0);
  svc.request.tilt_velocity = 20.0;

  if( m_clientServices["/pan_tilt_control/sweep"].call(svc) )
  {
    ROS_INFO("Continuous sweeping.");
  }
  else
  {
    ROS_ERROR("Failed to sweep.");
  }
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Topic Publishers
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void PanTiltTeleop::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "/pan_tilt_control/joint_command";
  m_publishers[strPub] =
    m_nh.advertise<trajectory_msgs::JointTrajectory>(strPub, nQueueDepth);

  strPub = "/xbox_360/rumble_command";
  m_publishers[strPub] =
    m_nh.advertise<hid::RumbleCmd>(strPub, nQueueDepth);
}

void PanTiltTeleop::publishJointCmd()
{
  static bool bPubJointCmd = false;

  if( m_msgJointTraj.joint_names.size() > 0 )
  {
    m_msgJointTraj.header.stamp    = ros::Time::now();
    m_msgJointTraj.header.frame_id = "0";
    m_msgJointTraj.header.seq++;
    m_msgJointTraj.points.push_back(m_msgJointTrajPoint);

    // publish
    m_publishers["/pan_tilt_control/joint_command"].publish(m_msgJointTraj);

    bPubJointCmd = true;
  }
  else if( bPubJointCmd )
  {
    freeze();
    bPubJointCmd = false;
  }
}

void PanTiltTeleop::publishRumbleCmd(int motorLeft, int motorRight)
{
  RumbleCmd msg;
  
  msg.left_rumble  = motorLeft;
  msg.right_rumble = motorRight;

  // publish
  m_publishers["/xbox_360/rumble_command"].publish(msg);
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Subscribed Topics
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void PanTiltTeleop::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "/pan_tilt_control/robot_status_ex";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &PanTiltTeleop::cbRobotStatus,
                                          &(*this));

  strSub = "/pan_tilt_control/joint_states_ex";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &PanTiltTeleop::cbJointState,
                                          &(*this));

  strSub = "/xbox_360/conn_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &PanTiltTeleop::cbXboxConnStatus,
                                          &(*this));

  strSub = "/xbox_360/controller_360_state";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &PanTiltTeleop::cbXboxBttnState,
                                          &(*this));
}

void PanTiltTeleop::cbRobotStatus(const RobotStatusExtended &msg)
{
  ROS_DEBUG("Received robot status.");

  // RDK might autoset pause/start here

  m_msgRobotStatus    = msg;
  m_bRcvdRobotStatus  = true;
  m_bHasRobotComm     = m_bRcvdJointState;
  m_nWdRobotCounter   = 0;
}

void PanTiltTeleop::cbJointState(const JointStateExtended &msg)
{
  static float  MaxEffort = 1023; // effort is between [0, 1023] 
  static float  DeadZone  = 256;  // no haptic feedback in this zone

  ssize_t   i;
  int       rumbleRight;

  ROS_DEBUG("Received joint state.");

  m_nWdRobotCounter = 0;
  m_msgJointState   = msg;

  if( !m_bRcvdJointState )
  {
    size_t  i;

    for(i=0; i<msg.name.size(); ++i)
    {
      m_mapJoints[msg.name[i]] = -1;
    }

    if( i > 0 )
    {
      m_bRcvdJointState = true;
    }
  }

#if 0
  //
  // Gripper tactile feedback.
  //
  if( (i = indexOfRobotJoint("grip")) >= 0 )
  {
    rumbleRight == ((float)msg.effort[i] - DeadZone)/(MaxEffort - DeadZone) *
                                                  XBOX360_RUMBLE_RIGHT_MAX;
    if( rumbleRight >= 0 )
    {
      setRumble(m_rumbleLeft, rumbleRight);
    }
  }
#endif
}

void PanTiltTeleop::cbXboxConnStatus(const hid::ConnStatus &msg)
{
  ROS_DEBUG("Received Xbox360 connectivity status.");

  m_bHasXboxComm    = msg.is_connected && msg.is_linked;
  m_nWdXboxCounter  = 0;

  m_msgConnStatus = msg;
}

void PanTiltTeleop::cbXboxBttnState(const hid::Controller360State &msg)
{
  ButtonState buttonState;

  ROS_DEBUG("Received Xbox360 button state.");

  if( m_bHasFullComm )
  {
    msgToState(msg, buttonState);

    switch( m_eState )
    {
      case TeleopStateReady:
        execAllButtonActions(buttonState);
        break;
      case TeleopStatePaused:
        buttonStart(buttonState);    // only active button in pause state
        break;
      case TeleopStateUninit:
      default:
        pause();
        break;
    }
  }

  m_buttonState = buttonState;
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Sanity
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void PanTiltTeleop::commCheck()
{
  if( m_bHasXboxComm )
  {
    if( ++m_nWdXboxCounter >= m_nWdXboxTimeout )
    {
      m_bHasXboxComm = false;
    }
  }

  if( m_bHasRobotComm )
  {
    if( ++m_nWdRobotCounter >= m_nWdRobotTimeout )
    {
      m_bHasRobotComm = false;
    }
  }

  bool hasComm  = m_bHasXboxComm && m_bHasRobotComm;

  // had communitcation, but no more
  if( m_bHasFullComm && !hasComm )
  {
    ROS_INFO("Lost communication with Xbox360 and/or PanTilt.");
    putRobotInSafeMode(m_msgConnStatus.is_connected);
  }

  m_bHasFullComm = hasComm;

  // not really a communication check function, but convenient.
  if( m_eState == TeleopStatePaused )
  {
    driveLEDsFigure8Pattern();
  }
}

void PanTiltTeleop::putRobotInSafeMode(bool bHard)
{
  static float  fGovDft = 0.20;

  // stop robot
  freeze();

  // set robot mode
  setRobotMode(PanTiltRobotModeAuto);
 
  if( bHard )
  {
    // nothing to do
  }
  
  m_eState = TeleopStateUninit;

  setLED(LEDPatOn);
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Xbox Actions
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void PanTiltTeleop::msgToState(const hid::Controller360State &msg,
                            ButtonState                   &buttonState)
{
  buttonState[ButtonIdContSweep]  = msg.a_button;
  buttonState[ButtonIdEStop]      = msg.b_button;
  buttonState[ButtonIdContPan]    = msg.x_button;
  buttonState[ButtonIdGotoZeroPt] = msg.y_button;

  buttonState[ButtonIdPause]      = msg.back_button;
  buttonState[ButtonIdToggleMode] = msg.center_button;
  buttonState[ButtonIdStart]      = msg.start_button;

  buttonState[ButtonIdTilt]       = msg.left_joy_y;
  buttonState[ButtonIdPan]        = msg.left_joy_x;
}

void PanTiltTeleop::execAllButtonActions(ButtonState &buttonState)
{
  clearJointTrajectory();

  // emergency stop
  buttonEStop(buttonState);

  //
  // Teleoperation state.
  //
  if( m_eState == TeleopStateReady )
  {
    buttonPause(buttonState);
  }
  else if( m_eState == TeleopStatePaused )
  {
    buttonStart(buttonState);
  }

  //
  // Moves.
  //
  if( canMove() )
  {
    buttonContinuousSweep(buttonState);
    buttonContinuousPan(buttonState);
    buttonGotoZeroPt(buttonState);

    buttonPan(buttonState);
    buttonTilt(buttonState);

    publishJointCmd();
  }
}

void PanTiltTeleop::buttonStart(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdStart, buttonState) )
  {
    ROS_INFO("Manual operation active, auto mode disabled.");

    setRobotMode(PanTiltRobotModeManual);

    if( m_msgRobotStatus.e_stopped.val == TriState::TRUE )
    {
      resetEStop();
    }

    ready();
  }
}

void PanTiltTeleop::buttonPause(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation paused, auto mode enabled.");

    setRobotMode(PanTiltRobotModeAuto);

    pause();
  }
}

void PanTiltTeleop::buttonEStop(ButtonState &buttonState)
{
  static int  clicks        = 0;            // number of button clicks
  static int  intvlCounter  = 0;            // intra-click interval counter
  static int  intvlTimeout  = countsPerSecond(0.3); // intra-click timeout

  //
  // Robot is estopped. This can come from a different node source. Make sure
  // counters are cleared.
  //
  if( m_msgRobotStatus.e_stopped.val == TriState::TRUE )
  {
    clicks = 0;
    intvlCounter = 0;
    return;
  }

  // button off to on
  if( buttonOffToOn(ButtonIdEStop, buttonState) )
  {
    ++clicks;
  }

  switch( clicks )
  {
    case 0:     // no click
      break;
    case 1:     // single click
      if( intvlCounter > intvlTimeout )
      {
        clicks = 0;
        intvlCounter = 0;
      }
      break;
    case 2:     // double click
      if( intvlCounter <= intvlTimeout )
      {
        estop();
        pause();
      }
      clicks = 0;
      intvlCounter = 0;
      break;
    default:    // multiple clicks
      clicks = 0;
      intvlCounter = 0;
      break;
  }
}

void PanTiltTeleop::buttonPan(ButtonState &buttonState)
{
  int     joy = buttonState[ButtonIdPan];
  int     i;
  double  pos;
  double  vel;

  // no joy
  if( joy == 0 )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("pan")) < 0 )
  {
    return;
  }

  pos = degToRad(40.0);
  vel = (double)(-joy) / (double)XBOX360_JOY_MAX * 30.0;

  if( vel < 0.0 )
  {
    pos = -pos;
    vel = -vel;
  }

  m_msgJointTrajPoint.positions[i] += pos;
  m_msgJointTrajPoint.velocities[i] = vel;
}

void PanTiltTeleop::buttonTilt(ButtonState &buttonState)
{
  int     joy = buttonState[ButtonIdTilt];
  int     i;
  double  pos;
  double  vel;

  // no joy
  if( joy == 0 )
  {
    return;
  }

  // no joint found on arm
  if( (i = addJointToTrajectoryPoint("tilt")) < 0 )
  {
    return;
  }

  pos = degToRad(40.0);
  vel = (double)(joy) / (double)XBOX360_JOY_MAX * 30.0;

  if( vel < 0.0 )
  {
    pos = -pos;
    vel = -vel;
  }

  m_msgJointTrajPoint.positions[i] += pos;
  m_msgJointTrajPoint.velocities[i] = vel;
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Support
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void PanTiltTeleop::pause()
{
  m_eState = TeleopStatePaused;

  setLED(LEDPatPaused);
}

void PanTiltTeleop::ready()
{
  m_eState = TeleopStateReady;

  setLED(LEDPatReady);
}

void PanTiltTeleop::driveLEDsFigure8Pattern()
{
  static int nLEDTimeout = -1;
  static int nLEDCounter = 0;
  static int iLED = 0;
  static int LEDPat[] =
  {
    XBOX360_LED_PAT_1_ON, XBOX360_LED_PAT_2_ON,
    XBOX360_LED_PAT_3_ON, XBOX360_LED_PAT_4_ON
  };

  // lazy init
  if( nLEDTimeout < 0 )
  {
    nLEDTimeout = countsPerSecond(0.25);
  }

  // switch pattern
  if( nLEDCounter++ >= nLEDTimeout )
  {
    iLED = (iLED + 1) % arraysize(LEDPat);
    setLED(LEDPat[iLED]);
    nLEDCounter = 0;
  }
}

ssize_t PanTiltTeleop::indexOfRobotJoint(const string &strJointName)
{
  for(size_t i=0; i<m_msgJointState.name.size(); ++i)
  {
    if( m_msgJointState.name[i] == strJointName )
    {
      return (ssize_t)i;
    }
  }

  return -1;
}

ssize_t PanTiltTeleop::indexOfTrajectoryJoint(const string &strJointName)
{
  for(size_t i=0; i<m_msgJointTraj.joint_names.size(); ++i)
  {
    if( m_msgJointTraj.joint_names[i] == strJointName )
    {
      return (ssize_t)i;
    }
  }

  return -1;
}

ssize_t PanTiltTeleop::addJointToTrajectoryPoint(const string &strJointName)
{
  MapJoints::iterator pos;
  ssize_t             i;

  pos = m_mapJoints.find(strJointName);

  // no joint
  if( pos == m_mapJoints.end() )
  {
    return -1;
  }

  // add new joint to trajectory point
  else if( pos->second < 0 )
  {
    m_msgJointTraj.joint_names.push_back(strJointName);
    if( (i = indexOfRobotJoint(strJointName)) >= 0 )
    {
      m_msgJointTrajPoint.positions.push_back(m_msgJointState.position[i]);
      m_msgJointTrajPoint.velocities.push_back(m_msgJointState.velocity[i]);
    }
    m_msgJointTrajPoint.accelerations.push_back(0.0);
    pos->second = m_msgJointTraj.joint_names.size() - 1;
    return pos->second;
  }

  // joint already added to trajectory point
  else
  {
    return pos->second;
  }
}

void PanTiltTeleop::clearJointTrajectory()
{
  MapJoints::iterator iter;

  m_msgJointTraj.joint_names.clear();
  m_msgJointTraj.points.clear();
  m_msgJointTrajPoint.positions.clear();
  m_msgJointTrajPoint.velocities.clear();
  m_msgJointTrajPoint.accelerations.clear();

  for(iter=m_mapJoints.begin(); iter!=m_mapJoints.end(); ++iter)
  {
    iter->second = -1;
  }
}
