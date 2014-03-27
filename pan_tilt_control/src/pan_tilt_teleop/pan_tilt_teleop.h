////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_teleop
//
// File:      pan_tilt_teleop.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS pan_tilt_teleop node class interface.
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

#ifndef _PAN_TILT_TELEOP_H
#define _PAN_TILT_TELEOP_H

//
// System
//
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
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


namespace pan_tilt_control
{
  /*!
   * \brief The class embodiment of the pan_tilt_teleop ROS node.
   */
  class PanTiltTeleop
  {
  public:
    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS client services type */
    typedef std::map<std::string, ros::ServiceClient> MapClientServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*! map of joint names to link indices */
    typedef std::map<std::string, int> MapJoints;

    /*!
     * \brief Teleoperation state.
     */
    enum TeleopState
    {
      TeleopStateUninit,    ///< not initialized
      TeleopStatePaused,    ///< paused
      TeleopStateReady      ///< ready and running
    };

    /*!
     * \brief Xbox360 button map ids.
     */
    enum ButtonId
    {
      ButtonIdContSweep     = rnr::Xbox360FeatIdAButton,  ///< continuous sweep
      ButtonIdEStop         = rnr::Xbox360FeatIdBButton,  ///< emergency stop
      ButtonIdContPan       = rnr::Xbox360FeatIdXButton,  ///< continuous pan
      ButtonIdGotoZeroPt    = rnr::Xbox360FeatIdYButton,  ///< goto zero point

      ButtonIdPause         = rnr::Xbox360FeatIdBack,     ///< pause teleop
      ButtonIdToggleMode    = rnr::Xbox360FeatIdCenterX,  ///< toggle op mode
      ButtonIdStart         = rnr::Xbox360FeatIdStart,    ///< start teleop

      ButtonIdTilt          = rnr::Xbox360FeatIdLeftJoyY, ///< tilt
      ButtonIdPan           = rnr::Xbox360FeatIdLeftJoyX, ///< pan
    };

    /*! teleop button state type */
    typedef std::map<int, int> ButtonState;

    /*!
     * \brief Xbox360 LED patterns.
     */
    enum LEDPat
    {
      LEDPatOff       = XBOX360_LED_PAT_ALL_OFF,    ///< all off
      LEDPatOn        = XBOX360_LED_PAT_ALL_BLINK,  ///< default xbox on pattern
      LEDPatPaused    = XBOX360_LED_PAT_4_ON,       ///< pause teleop
      LEDPatReady     = XBOX360_LED_PAT_ALL_SPIN    ///< ready to teleop
    };

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    PanTiltTeleop(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~PanTiltTeleop();

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices()
    {
      // none
    }

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices();

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish()
    {
      // No periodic publishing. All event driven.
    }

    /*!
     * \brief Check communications.
     *
     * Call in main loop.
     */
    virtual void commCheck();

    /*!
     * \brief Put robot into safe mode.
     *
     * \param bHard   Harden safe mode. When teleop node dies or xbox is 
     *                physically disconnected, robot is set to known defaults.
     */
    void putRobotInSafeMode(bool bHard);

    /*!
     * \brief Test if robot can move.
     *
     * \return Returns true or false.
     */
    bool canMove()
    {
      if((m_eState == TeleopStateReady) &&
         (m_msgRobotStatus.is_calibrated.val ==
                                       industrial_msgs::TriState::TRUE) &&
         (m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::FALSE) &&
         (m_msgRobotStatus.in_error.val == industrial_msgs::TriState::FALSE))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Convert seconds to loop counts.
     *
     * \param seconds Seconds.
     *
     * \return Count.
     */
    int countsPerSecond(double seconds)
    {
      return (int)(seconds * m_hz);
    }

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    double            m_hz;       ///< application nominal loop rate

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< teleop server services
    MapClientServices m_clientServices; ///< teleop client services
    MapPublishers     m_publishers;     ///< teleop publishers
    MapSubscriptions  m_subscriptions;  ///< teleop subscriptions

    // state
    TeleopState       m_eState;         ///< teleoperation state
    bool              m_bHasXboxComm;   ///< Xbox communications is [not] good
    int               m_nWdXboxCounter; ///< Xbox watchdog counter
    int               m_nWdXboxTimeout; ///< Xbox watchdog timeout
    bool              m_bHasRobotComm;  ///< robot communications is [not] good
    int               m_nWdRobotCounter;///< robot watchdog counter
    int               m_nWdRobotTimeout;///< robot watchdog timeout
    bool              m_bRcvdRobotStatus; ///< has [not] recieved any bot status
    bool              m_bRcvdJointState; ///< has [not] recieved any joint state
    bool              m_bHasFullComm;   ///< good full communications
    ButtonState       m_buttonState;    ///< saved previous button state
    int               m_rumbleLeft;     ///< saved previous left rumble speed 
    int               m_rumbleRight;    ///< saved previous right rumble speed 
    MapJoints         m_mapJoints;      ///< map of joint names - link indices

    // messages
    pan_tilt_control::RobotStatusExtended m_msgRobotStatus;
                                                    ///< current robot status 
    pan_tilt_control::JointStateExtended m_msgJointState;
                                                    ///< current joint state
    trajectory_msgs::JointTrajectory m_msgJointTraj;
                                                    ///< working joint traj.
    trajectory_msgs::JointTrajectoryPoint m_msgJointTrajPoint;
                                                    ///< working joint traj. pt
    hid::ConnStatus m_msgConnStatus;                ///< saved last conn status 


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Server Service callbacks
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    // none


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Client Services
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Set Xbox360 LED pattern client service.
     *
     * \param pattern   LED pattern.
     */
    void setLED(int pattern);

    /*!
     * \brief Set Xbox360 rumble motors client service.
     *
     * \param motorLeft   Left motor speed.
     * \param motorRight  Right motor speed.
     */
    void setRumble(int motorLeft, int motorRight);

    /*!
     * \brief Emergency stop robot client service.
     */
    void estop();

    /*!
     * \brief Freeze (stop) robot client service.
     */
    void freeze();

    /*!
     * \brief Move robot to zero point position client service.
     */
    void gotoZeroPt();

    /*!
     * \brief Continuously pan.
     */
    void pan();

    /*!
     * \brief Reset emergency stop client service.
     */
    void resetEStop();

    /*!
     * \brief Set robot mode client service.
     *
     * \param mode    Auto or manual mode.
     */
    void setRobotMode(pan_tilt::PanTiltRobotMode mode);

    /*!
     * \brief Continuosly sweep (pan and tilt).
     */
    void sweep();


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Topic Publishers
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Publish brake command.
     */
    void publishJointCmd();

    /*!
     * \brief Publish Xbox360 rumble command.
     */
    void publishRumbleCmd(int motorLeft, int motorRight);


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Subscribed Topic Callbacks
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Robot status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbRobotStatus(const pan_tilt_control::RobotStatusExtended &msg);

    /*!
     * \brief Joint state callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbJointState(const pan_tilt_control::JointStateExtended &msg);
    
    /*!
     * \brief Xbox360 HID connectivity status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbXboxConnStatus(const hid::ConnStatus &msg);

    /*!
     * \brief Xbox360 HID button state callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbXboxBttnState(const hid::Controller360State &msg);


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Xbox Actions
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Convert Xbox360 controller message to button state.
     *
     * \param msg           Message.
     * \param buttonState   New button state.
     */
    void msgToState(const hid::Controller360State &msg,
                    ButtonState                   &buttonState);

    /*!
     * \brief Test if button transitions from off to on.
     *
     * \param id            Button id.
     * \param buttonState   New button state.
     *
     * \return Returns true or false.
     */
    bool buttonOffToOn(int id, ButtonState &buttonState)
    {
      return (m_buttonState[id] == 0) && (buttonState[id] == 1);
    }

    /*!
     * \brief Test if button state is differenet.
     *
     * \param id            Button id.
     * \param buttonState   New button state.
     *
     * \return Returns true or false.
     */
    bool buttonDiff(int id, ButtonState &buttonState)
    {
      return m_buttonState[id] != buttonState[id];
    }

    /*!
     * \brief Execute all button actions.
     *
     * \param buttonState   New button state.
     */
    void execAllButtonActions(ButtonState &buttonState);

    /*!
     * \brief Execute start button action.
     *
     * \param buttonState   New button state.
     */
    void buttonStart(ButtonState &buttonState);

    /*!
     * \brief Execute pause button action.
     *
     * \param buttonState   New button state.
     */
    void buttonPause(ButtonState &buttonState);

    /*!
     * \brief Execute emergency stop button action.
     *
     * \param buttonState   New button state.
     */
    void buttonEStop(ButtonState &buttonState);

    /*!
     * \brief Execute continuous pan button action.
     *
     * \param buttonState   New button state.
     */
    void buttonContinuousPan(ButtonState &buttonState)
    {
      if( buttonOffToOn(ButtonIdContPan, buttonState) )
      {
        pan();
      }
    }

    /*!
     * \brief Execute continuous sweep button action.
     *
     * \param buttonState   New button state.
     */
    void buttonContinuousSweep(ButtonState &buttonState)
    {
      if( buttonOffToOn(ButtonIdContSweep, buttonState) )
      {
        sweep();
      }
    }

    /*!
     * \brief Execute move to zero point button action.
     *
     * \param buttonState   New button state.
     */
    void buttonGotoZeroPt(ButtonState &buttonState)
    {
      if( buttonOffToOn(ButtonIdGotoZeroPt, buttonState) )
      {
        gotoZeroPt();
      }
    }

    /*!
     * \brief Execute pan move button action.
     *
     * \param buttonState   New button state.
     */
    void buttonPan(ButtonState &buttonState);

    /*!
     * \brief Execute tilt move button action.
     *
     * \param buttonState   New button state.
     */
    void buttonTilt(ButtonState &buttonState);


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Support 
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Go to pause teleoperation state.
     */
    void pause();

    /*!
     * \brief Go to ready to teleoperate state.
     */
    void ready();

    /*!
     * \brief Drive Xbox360 LEDs into a figure 8 pattern.
     */
    void driveLEDsFigure8Pattern();

    /*!
     * \brief Get the index of the joint of the robot.
     *
     * \param strJointName    Joint name.
     * 
     * \return Returns index \h_ge 0 if found. Otherwise returns -1.
     */
    ssize_t indexOfRobotJoint(const std::string &strJointName);

    /*!
     * \brief Get the index of the joint in the current joint trajectory.
     *
     * \param strJointName    Joint name.
     * 
     * \return Returns index \h_ge 0 if found. Otherwise returns -1.
     */
    ssize_t indexOfTrajectoryJoint(const std::string &strJointName);
    
    /*!
     * \brief Add a joint to the current joint trajectory point.
     *
     *  The joint is only added if it does not exist in point.
     *
     * \param strJointName    Joint name.
     * 
     * \return Returns index \h_ge 0 of the added or existing joint.
     * If the joint does not exists in the arm, -1 is returned.
     */
    ssize_t addJointToTrajectoryPoint(const std::string &strJointName);

    /*!
     * \brief Clear joint trajectory.
     */
    void clearJointTrajectory();
    
  };

} // namespace pan_tilt_control


#endif // _PAN_TILT_TELEOP_H
