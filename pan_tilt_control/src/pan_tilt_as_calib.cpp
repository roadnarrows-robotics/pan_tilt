////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_control
//
// File:      pan_tilt_as_calib.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt calibration action server class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Danial Packard (daniel@roadnarrows.com)
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

#include <unistd.h>

#include <boost/bind.hpp>

//
// ROS
//
#include "ros/ros.h"

//
// ROS generated core, industrial, and pan-tilt messages.
//
#include "std_msgs/String.h"
#include "pan_tilt_control/OpState.h"
#include "pan_tilt_control/JointStateExtended.h"

//
// ROS generated action servers.
//
#include "actionlib/server/simple_action_server.h"
#include "pan_tilt_control/CalibrateAction.h"

//
// RoadNarrows embedded pan-tilt library.
//
#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptJoint.h"
#include "pan_tilt/ptRobot.h"

//
// Node headers.
//
#include "pan_tilt_control.h"
#include "pan_tilt_as_calib.h"


using namespace std;
using namespace pan_tilt;


void ASCalibrate::execute_cb(
                          const pan_tilt_control::CalibrateGoalConstPtr &goal)
{
  PanTiltRobot           &robot(pantilt_.getRobot());
  PanTiltJointStatePoint  state;
  int                     rc;

  ROS_INFO("%s: Executing calibrate action - please standby.",
      action_name_.c_str());

  //
  // Start pan-tilt to calibrate asynchronously (i.e. non-blocking).
  //
  rc = robot.calibrateAsync(goal->force_recalib? true: false);

  if( rc != PT_OK )
  {
    ROS_ERROR("%s: Failed to initiate calibration.", action_name_.c_str());
    result_.op.calib_state = pan_tilt_control::OpState::UNCALIBRATED;
    as_.setAborted(result_);
    return;
  }

  ros::Rate r(2);

  while( robot.getAsyncState() == PanTiltAsyncTaskStateWorking )
  {
    //
    // Action was preempted.
    //
    if( as_.isPreemptRequested() || !ros::ok() )
    {
      ROS_INFO("%s: Execution preempted.", action_name_.c_str());
      result_.op.calib_state = pan_tilt_control::OpState::UNCALIBRATED;
      as_.setPreempted(result_); // set the action state to preempted
      return;
    }

    //
    // Keep providing feedback during calibration.
    //
    else
    {
      robot.getJointState(state);
      pantilt_.updateExtendedJointStateMsg(state, feedback_.joint);
      as_.publishFeedback(feedback_);
      r.sleep();
    }
  }

  // check outcome of asynchronous task
  rc = robot.getAsyncRc();

  // got calibrated
  if( (rc == PT_OK) && robot.isCalibrated() )
  {
    ROS_INFO("%s: Calibration succeeded.", action_name_.c_str());
    result_.op.calib_state = pan_tilt_control::OpState::CALIBRATED;
    as_.setSucceeded(result_);
  }
  // nope
  else
  {
    ROS_ERROR("Calibration failed with error code %d.", rc);
    result_.op.calib_state = pan_tilt_control::OpState::UNCALIBRATED;
    as_.setAborted(result_);
  }
}

void ASCalibrate::preempt_cb()
{
  ROS_INFO("%s: Preempt calibration.", action_name_.c_str());
  pantilt_.getRobot().cancelAsyncTask();
  //RDK as_.setPreempted(); need this?
  //RDK as_.acceptNewGoal(); does return from execution autoset this state?
}
