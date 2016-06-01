////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptTune.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt tuning
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows LLC
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

#ifndef _PAN_TILT_TUNE_H
#define _PAN_TILT_TUNE_H

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"

/*!
 *  \brief The pan_tilt namespace encapsulates all pan-tilt related constructs.
 */
namespace pan_tilt
{
  /*!
   * \brief Trajectory action server control rate (Hertz).
   *
   * Higher Hertz will give better trajectory control but will incur more CPU
   * load.
   *
   * - units:       Hertz
   * - minimum:     \> 0
   * - maximum:     goforit
   * - recommended: 60
   */
  const double PtTuneTrajHz = 60.0;
  
  /*!
   * \brief Trajectory action server position norm.
   *
   * - one of:      \ref PanTiltNormL1, \ref PanTiltNormL2, \ref PanTiltNormLinf
   * - recommended: PanTiltNormL1
   */
  const pan_tilt::PanTiltNorm PtTuneTrajNorm = pan_tilt::PanTiltNormL1;
  
  /*!
   * \brief Trajectory intermediate waypoint distance epsilon (radians).
   *
   * Trajectory waypoints distances within epsilon are considered reached.
   *
   * \verbatim
   * ||WP-P|| < epsilon <==> waypoint reached
   *   Where WP is the joint goal vector of positions and P is the vector
   *   of joint positions.
   * \endverbatim
   * 
   * - units:       radians
   * - minimum:     0.0
   * - maximum:     none
   * - recommended: 10.0 degrees equivalent
   */
  const double PtTuneTrajEpsilonWp = pan_tilt::degToRad(10.0);
  
  /*!
   * \brief Trajectory endpoint distance epsilon (radians).
   *
   * Trajectory endpoint distance within epsilon are considered reached.
   *
   * \verbatim
   * ||EP-P|| < epsilon <==> waypoint reached
   *   Where EP is the joint endpoint goal vector of positions and P is the
   *   vector of joint positions.
   * \endverbatim
   * 
   * - units:       radians
   * - minimum:     0.0
   * - maximum:     none
   * - recommended: 2.0 degrees equivalent
   */
  const double PtTuneTrajEpsilonEp = pan_tilt::degToRad(2.0);
  
  /*!
   * \brief Trajectory action server joint non-zero velocity.
   *
   * Any joint velocities below this threshold are considered zero velocity.
   * Motors running very slowly often perform poorly. The Dynamixel servos are
   * no exception.
   *
   * - units:       radians/second
   * - minimum:     0.0
   * - maximum:     small positive number
   * - recommended: 0.2 degrees/second equivalent
   */
  const double PtTuneTrajNonZeroVel = pan_tilt::degToRad(0.2);
  
  /*!
   * \brief Trajectory action server joint minimum velocity.
   *
   * Any joint velocities below this threshold that are non-zero (see above)
   * will be set to this minimum.
   *
   * - units:       radians/second
   * - minimum:     0.0
   * - maximum:     joint maximum velocity
   * - recommended: 5.0 degrees/second equivalent
   */
  const double PtTuneTrajMinVel    = pan_tilt::degToRad(5.0);
  
  /*!
   * \brief Trajectory action server joint velocity derate.
   *
   * All velocities specified in the trajectory are derate by this value.
   *
   * - units:       unitless
   * - minimum:     \> 0.0
   * - maximum:     1.0
   * - recommended: 1.0 (no derate)
   */
  const double PtTuneTrajVelDerate = 1.0;
  
  /*!
   * \brief Trajectory action server joint waypoint overshoot.
   *
   * Here is the problem. Dynamixel servo firmware uses a position PID to stop
   * at a goal waypoint. This can result in a jerky trajectory, with the action
   * server trying to detect when a waypoint has been reached and then quickly
   * load the next waypoint all the while the servo firmware is trying to stop
   * to servo.
   *
   * The endpoint (last waypoint) does not use overshoot.
   *
   * - units:       radians
   * - minimum:     0.0
   * - maximum:     sanity
   * - recommended: 8.0 degrees equivalent
   */
  const double PtTuneTrajWpPosOver = pan_tilt::degToRad(8.0);

} // namespace pan_tilt

#endif // _PAN_TILT_TUNE_H

