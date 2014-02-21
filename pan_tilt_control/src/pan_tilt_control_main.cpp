////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/pan_tilt
//
// ROS Node:  pan_tilt_control
//
// File:      pan_tilt_control_main.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS pan_tilt_control main.
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


#include <string>

//
// ROS 
//
#include "ros/ros.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

//
// Node headers.
//
#include "pan_tilt_control.h"
#include "pan_tilt_as_calib.h"

using namespace ::std;
using namespace pan_tilt;

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error

const char *NodeName = "pan_tilt_control";  ///< this ROS node's name


/*!
 *  \brief ROS pan-tilt control node main.
 *
 *  \param
 *  \param
 *
 *  \return
 */
int main(int argc, char *argv[])
{
  string  strNodeName;  // ROS-given node name
  int     rc;

  //
  // 
  //
  ros::init(argc, argv, NodeName);

  //
  //
  // A ctrl-c interrupt will stop attempts to connect to the ROS core.
  //
  ros::NodeHandle nh(NodeName);

  // actual ROS-given node name
  strNodeName = ros::this_node::getName();

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    // add optional non-ROS unit tests here, then simply exit.
    return APP_EC_OK;
  }

  ROS_INFO("%s: Node started.", strNodeName.c_str());
  
  //
  // Set log level for RN libs. All RoadNarrows software packages use logging
  // mechanism supported in librnr.
  //
  // TODO Set threshold from a given ros option.
  //
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  //
  // Create a pan-tilt node object.
  //
  PanTiltControl  pantilt(nh);

  //
  // Connect to the pan-tilt mechanism.
  //
  if( (rc = pantilt.connect()) != PT_OK )
  {
    ROS_ERROR_STREAM(strNodeName << ": Failed to connect to pan-tilt.");
    return APP_EC_INIT;
  }

  //
  // Advertise services.
  //
  pantilt.advertiseServices();

  ROS_INFO("%s: Services registered.", strNodeName.c_str());

  //
  // Advertise publishers.
  //
  pantilt.advertisePublishers();
  
  ROS_INFO("%s: Publishers registered.", strNodeName.c_str());
  

  //
  // Subscribed to topics.
  //
  pantilt.subscribeToTopics();
  
  ROS_INFO("%s: Subscribed topics registered.", strNodeName.c_str());

  //
  // Create Action Servers
  //
  ASCalibrate asCalib("calibrate_as", pantilt);

  ROS_INFO("%s: Action servers created.", strNodeName.c_str());

  // set loop rate in Hertz
  ros::Rate loop_rate(5);

  ROS_INFO("%s: Ready.", strNodeName.c_str());

  //
  //
  //
  while( ros::ok() )
  {
    // 
    ros::spinOnce(); 

    // 
    pantilt.publish();

    // 
    loop_rate.sleep();
  }

  return APP_EC_OK;
}
