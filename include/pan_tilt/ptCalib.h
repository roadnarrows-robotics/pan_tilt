////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptCalib.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltCalib - Pan-Tilt calibration class interface.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.RoadNarrows.com)
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

#ifndef _PT_CALIB_H
#define _PT_CALIB_H

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptJoint.h"


namespace pan_tilt
{
  //
  // Forward declaraions.
  //
  class PanTiltRobot;

  /*!
   * \brief Pan-Tilt robotic manipulator calibration by stretching class.
   *
   * The pan-tilt will do a choreographed sequence of moves to find the limits
   * or top-dead-centers for all joints.
   *
   * \note It is best if the user places the arm in the upright position and
   * with not obstructions in the workspace prior to executing sequence.
   *
   * \par Move Sequence:
   * \li Tilt clockwise and counter-clockwise limits.
   * \li Pan clockwise and counter-clockwise limits.
   */
  class PanTiltCalib
  {
  public: 

    /*!
     * \breif Default initialization constructor.
     *
     * \param robot   Instance of pan-tilt robot.
     */
    PanTiltCalib(PanTiltRobot &robot) : m_robot(robot)
    {
    }
  
    /*!
     * \breif Destructor.
     */
    virtual ~PanTiltCalib()
    {
    }

    /*!
     * \brief Calibrate the pan-tilt's robotic mechanism.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrate();

  protected:
    PanTiltRobot &m_robot;    ///< robot instance

    /*!
     * \brief Calibrate joint by torque limits.
     *
     * The joint is rotated to its minimum and maximum physical end
     * points. When the torque reaches a software defined limit, an endpoint
     * is considered reached.
     *
     * From the joint specification and detected limits, the home position
     * at 0\h_deg is fine tuned.
     *
     * When the joint calibration moves are finished, the joint is repositioned
     * at its new home zero point position.
     *
     * \par Heuristic:
     *  -# Rotate in minimum direction until torque limit reached.
     *  -# Mark position.
     *  -# Rotate in maximum direction until torque limit reached.
     *  -# Mark position.
     *  -# Calculate 0\h_deg based on expected min,max positions and detected
     *     min,max positions.
     *  -# Rotate to 0\h_deg.
     *  -# Reset odomter.
     *
     * \param joint   Robotic joint to calibrate.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrateJointByTorqueLimits(PanTiltRobotJoint &joint);

    /*!
     * \brief Move servo to position.
     *
     * This call blocks until the move is complete.
     *
     * \param pServo      Pointer to servo.
     * \param nOdGoalPos  Goal position in raw odometer units.
     * \param nSpeed      Speed in raw units.
     *
     * \return New odometer position.
     */
    virtual int moveWait(DynaServo *pServo, int nOdGoalPos, int nSpeed);
  };

} // namespace pan_tilt

#endif // _PT_CALIB_H
