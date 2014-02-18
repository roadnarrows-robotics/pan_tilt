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
 * \brief The ROS pan_tilt_control node class interface.
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

#ifndef _PAN_TILT_CONTROL_H
#define _PAN_TILT_CONTROL_H

#include <string>
#include <map>

//
// Includes for boost libraries
//
#include <boost/bind.hpp>

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
#include "pan_tilt_control/ClearAlarms.h"
#include "pan_tilt_control/EStop.h"
#include "pan_tilt_control/Freeze.h"
#include "pan_tilt_control/GetProductInfo.h"
#include "pan_tilt_control/GotoZeroPt.h"
#include "pan_tilt_control/IsAlarmed.h"
#include "pan_tilt_control/IsCalibrated.h"
#include "pan_tilt_control/Release.h"
#include "pan_tilt_control/ResetEStop.h"
#include "pan_tilt_control/SetRobotMode.h"
#include "pan_tilt_control/Stop.h"

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


namespace pan_tilt
{
  /*!
   * \brief The class embodiment of the pan_tilt_control ROS node.
   */
  class PanTiltControl
  {
  public:
    /*! map of ROS services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    PanTiltControl(ros::NodeHandle &nh);

    virtual ~PanTiltControl();

    virtual int connect(const std::string &strDev="/dev/ttyUSB0",
                        const int          nBaudRate=1000000)
    {
      return m_robot.connect(strDev, nBaudRate);
    }

    virtual int disconnect()
    {
      return m_robot.disconnect();
    }

    virtual void advertiseServices();

    virtual void advertisePublishers(int nQueueDepth=10);

    virtual void subscribeToTopics(int nQueueDepth=10);

    virtual void publish();

    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    PanTiltRobot &getRobot()
    {
      return m_robot;
    }

    void updateJointStateMsg(PanTiltJointStatePoint &state,
                             sensor_msgs::JointState &msg);

    void updateExtendedJointStateMsg(PanTiltJointStatePoint &state,
                                     pan_tilt_control::JointStateExtended &msg);

    void updateRobotStatusMsg(PanTiltRobotStatus &status,
                              industrial_msgs::RobotStatus &msg);

    void updateExtendedRobotStatusMsg(PanTiltRobotStatus &status,
                                   pan_tilt_control::RobotStatusExtended &msg);

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    PanTiltRobot     m_robot;     ///< real-time, pan-tilt robot mechanism

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< pan-tilt control services
    MapPublishers     m_publishers;     ///< pan-tilt control publishers
    MapSubscriptions  m_subscriptions;  ///< pan-tilt control subscriptions

    // Messages for published data.
    sensor_msgs::JointState               m_msgJointState;
                                              ///< joint state message
    pan_tilt_control::JointStateExtended  m_msgJointStateEx;
                                              ///< extended joint state message
    industrial_msgs::RobotStatus          m_msgRobotStatus;
                                              ///< robot status message
    pan_tilt_control::RobotStatusExtended m_msgRobotStatusEx;
                                              ///< extended robot status message

    //..........................................................................
    // Service callbacks
    //..........................................................................

    bool clearAlarms(pan_tilt_control::ClearAlarms::Request  &req,
                     pan_tilt_control::ClearAlarms::Response &rsp);

    bool estop(pan_tilt_control::EStop::Request  &req,
               pan_tilt_control::EStop::Response &rsp);

    bool freeze(pan_tilt_control::Freeze::Request  &req,
                pan_tilt_control::Freeze::Response &rsp);

    bool getProductInfo(pan_tilt_control::GetProductInfo::Request  &req,
                        pan_tilt_control::GetProductInfo::Response &rsp);

    bool gotoZeroPt(pan_tilt_control::GotoZeroPt::Request  &req,
                    pan_tilt_control::GotoZeroPt::Response &rsp);

    bool isAlarmed(pan_tilt_control::IsAlarmed::Request  &req,
                   pan_tilt_control::IsAlarmed::Response &rsp);

    bool isCalibrated(pan_tilt_control::IsCalibrated::Request  &req,
                      pan_tilt_control::IsCalibrated::Response &rsp);

    bool release(pan_tilt_control::Release::Request  &req,
                 pan_tilt_control::Release::Response &rsp);

    bool resetEStop(pan_tilt_control::ResetEStop::Request  &req,
                    pan_tilt_control::ResetEStop::Response &rsp);

    bool setRobotMode(pan_tilt_control::SetRobotMode::Request  &req,
                      pan_tilt_control::SetRobotMode::Response &rsp);

    bool stop(pan_tilt_control::Stop::Request  &req,
              pan_tilt_control::Stop::Response &rsp);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    void publishJointState();

    void publishRobotStatus();


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    void execJointCmd(const trajectory_msgs::JointTrajectory &jt);
  };

} // namespace pan_tilt


#endif // _PAN_TILT_CONTROL_H
