////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_control
//
// File:      pan_tilt_as_follow_traj.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt follow joint trajectory action server class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows LLC
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

#ifndef _PAN_TILT_AS_TRAJECTORY_H
#define _PAN_TILT_AS_TRAJECTORY_H

//
// System and Boost
//
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>

//
// ROS
//
#include "ros/ros.h"

//
// ROS generated core, industrial, and pan_tilt messages.
//
#include "std_msgs/String.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

//
// ROS generated action servers.
//
#include "actionlib/server/simple_action_server.h"

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
   * \brief Pan-Tilt follow joint trajectory action server class.
   *
   * A trajectory is a series of waypoints defining the path for the Pan-Tilt
   * to follow. A waypoint contains two vectors of joint positions P and
   * velocities V.  P specifies the waypoint location in joint space, while V
   * specifies the target velocities _at_ the waypoint.
   *
   * From MoveIt!:
   * \li The first waypoint is always the current arm location with V = 0.
   * \li The endpoint (last waypoint) is the final destination and also has
   * V = 0.
   */
  class ASFollowTrajectory
  {
  public:
    static const double MaxSecs = 10.0; ///< maximum seconds to reach a waypoint

    /*!
     * \brief Trajectory execution states.
     */
    enum ExecState
    {
      ExecStateStartMove,     ///< start move to next waypoint
      ExecStateMonitorMove,   ///< monitor move
      ExecStateTerminate      ///< terminate 
    };

    /*!
     * \brief Initialization constructor.
     *
     * \param name    Action server name.
     * \param node    Node-specific class instance.
     */
    ASFollowTrajectory(std::string name, PanTiltControl &node) :
      action_name_(name),       // action name
      node_(node),              // pan_tilt node
      m_robot(node.getRobot()),
      as_(node.getNodeHandle(), // simple action server
          name,                       // action name
          boost::bind(&ASFollowTrajectory::execute_cb, this, _1),
                                      // execute callback
          false)                      // don't auto-start
    {
      // 
      // Optionally register the goal and feeback callbacks
      //
      as_.registerPreemptCallback(boost::bind(&ASFollowTrajectory::preempt_cb,
                                  this));

      // start the action server
      start();
    }

    /*!
     * \brief Destructor.
     */
    virtual ~ASFollowTrajectory()
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
     *
     * \param goal  Goal joint trajectory path.
     */
    void execute_cb(
                  const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    /*!
     * \brief ROS callback to preempt action.
     *
     * This is only needed if actions are required outside of the blocking
     * execution callback thread.
     */
    void preempt_cb();

  protected:
    // action server base
    std::string       action_name_;   ///< action name
    PanTiltControl &node_;  ///< pan_tilt control node instance
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                                      as_;  ///< simple action server
    control_msgs::FollowJointTrajectoryFeedback feedback_;
                                    ///< progress feedback
    control_msgs::FollowJointTrajectoryResult result_;
                                    ///< action results

    // pan_tilt robot and parameters
    pan_tilt::PanTiltRobot &m_robot;        ///< pan_tilt robot
    pan_tilt::PanTiltNorm   m_eNorm;        ///< joint point distance norm
    double                  m_fEpsilonWp;   ///< waypoint distance epsilon
    double                  m_fEpsilonEp;   ///< endpoint distance epsilon

    // goal trajectory
    trajectory_msgs::JointTrajectory m_trajGoal;  ///< goal trajectory
    trajectory_msgs::JointTrajectory m_trajTrans; ///< transformed trajectory
    ssize_t       m_iNumWaypoints;                ///< number of goal waypoints
    ssize_t       m_iEndpoint;                    ///< endpoint index

    // state
    ExecState     m_eState;             ///< execution state
    bool          m_bTrajCompleted;     ///< trajectory [not] completed to end
    int           m_nMaxIters;          ///< maximum iterations per waypoint
    int           m_iterMonitor;        ///< monitoring iteration count
    double        m_fWorstJointDist;    ///< worst joint distance
    std::string   m_strWorstJointName;  ///< worst joint name

    /*!
     * \brief Get the next significant waypoint.
     *
     * \param iWaypoint   Current waypoint along the trajectoy path.
     */
    ssize_t nextWaypoint(ssize_t iCurpoint);

    /*!
     * \brief Groom waypoint before issuing a move to the waypoint.
     *
     * Pan-Tilt Constraints:
     * \li A minimum velocity is required for non-zero velocity joints in order
     *      for movement to occur.
     * \li A maximum velocity is required to protect the robot.
     * \li Since the endpoint has V=0, fix-up so that the arm can get there!
     *
     * \param iWaypoint   Current waypoint along the trajectoy path.
     */
    void groomWaypoint(ssize_t iWaypoint);

    /*!
     * \brief Start move to next waypoint or endpoint.
     *
     * \param iWaypoint   Next waypoint along the trajectoy path.
     *
     * \return Returns next execution state.
     */
    ExecState startMoveToPoint(ssize_t iWaypoint);

    /*!
     * \brief Monitor move to intermediate waypoint.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns next execution state.
     */
    ExecState monitorMoveToWaypoint(ssize_t iWaypoint);

    /*!
     * \brief Monitor move to endpoint (last waypoint).
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Returns next execution state.
     */
    ExecState monitorMoveToEndpoint(ssize_t iWaypoint);

    /*!
     * \brief Test if current move to the the target waypoint failed.
     *
     * \return Returns true or false.
     */
    bool failedWaypoint();

    /*!
     * \brief Measure waypoint distance from the current robot position.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Distance from waypoint.
     */
    double measureDist(ssize_t iWaypoint);

    /*!
     * \brief Measure waypoint move from current position and provide feedback.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     *
     * \return Distance from waypoint.
     */
    double provideFeedback(ssize_t iWaypoint);

    /*!
     * \brief Publish feedback.
     *
     * \param iWaypoint Current waypoint along the trajectoy path.
     */
    void publishFeedback(ssize_t iWaypoint);

    /*!
     * \brief Clear feedback data.
     */
    void clearFeedback();

    /*!
     * \brief Add point to feedback.
     *
     * \param jointName     Joint name.
     * \param jointWpPos    Joint waypoint position (radians).
     * \param jointWpVel    Joint waypoint velocity (radians/second).
     * \param jointCurPos   Joint current position (radians).
     * \param jointCurVel   Joint current velocity (radians/second).
     */
    void addFeedbackPoint(const std::string &jointName,
                          const double jointWpPos,  const double jointWpVel,
                          const double jointCurPos, const double jointCurVel);
  };

} // namespace pan_tilt_control

#endif // _PAN_TILT_AS_TRAJECTORY_H
