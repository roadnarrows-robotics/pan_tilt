////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_control
//
// File:      pan_tilt_as_calib.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Pan-Tilt calibration action server class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Danial Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2016  RoadNarrows LLC
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

#ifndef _PAN_TILT_AS_CALIB_H
#define _PAN_TILT_AS_CALIB_H

//
// System and Boost
//
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
#include "sensor_msgs/JointState.h"
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
#include "pan_tilt/ptRobot.h"

//
// Node headers.
//
#include "pan_tilt_control.h"


namespace pan_tilt_control
{
  /*!
   * \brief Calibrate the pan-tilt robotic mechanism action server class.
   */
  class ASCalibrate
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param name      Action server name.
     * \param pantilt   Node-specific class instance.
     */
    ASCalibrate(std::string name, PanTiltControl &pantilt) :
      action_name_(name),           // action name
      pantilt_(pantilt),            // pan-tilt node
      as_(pantilt.getNodeHandle(),  // simple action server
          name,                       // action name
          boost::bind(&ASCalibrate::execute_cb, this, _1),
                                      // execute callback
          false)                      // don't auto-start
    {
      // 
      // Optionally register the goal and feeback callbacks
      //
      as_.registerPreemptCallback(boost::bind(&ASCalibrate::preempt_cb, this));

      // start the action server
      start();
    }

    /*!
     * \brief Destructor.
     */
    virtual ~ASCalibrate()
    {
    }

    /*!
     * \brief Start the action server.
     */
    void start()
    {
      as_.start();
    }

    /*!
     * \brief ROS callback to execute action.
     *
     * The callback is executed in a separate ROS-created thread so that it
     * can block.Typically, the callback is invoked within a ROS spin.
     */
    void execute_cb(const pan_tilt_control::CalibrateGoalConstPtr &goal);

    /*!
     * \brief ROS callback to preempt action.
     *
     * This is only needed if actions are required outside of the blocking
     * execution callback thread.
     */
    void preempt_cb();

  protected:
    std::string     action_name_;               ///< action name
    PanTiltControl &pantilt_;                   ///< pan-tilt control instance

    actionlib::SimpleActionServer<pan_tilt_control::CalibrateAction> as_;
                                                    ///< action simple server
    pan_tilt_control::CalibrateFeedback feedback_;  ///< progress feedback
    pan_tilt_control::CalibrateResult   result_;    ///< action results
  };

} // namespace pan_tilt_control

#endif // _PAN_TILT_AS_CALIB_H
