////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptJoint.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt robot joint classes interfaces.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
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

#ifndef _PT_JOINT_H
#define _PT_JOINT_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "pan_tilt/pan_tilt.h"

namespace pan_tilt
{
  //
  // Forward declarations
  //
  class PanTiltRobot;

  // ---------------------------------------------------------------------------
  // Class PanTiltRobotJoint
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Operational robotic joint class.
   *
   * This class contains all of the data necessary to control the operation
   * of an pan-tilt robotic joint.
   */
  class PanTiltRobotJoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltRobotJoint();

    /*!
     * \brief Copy constructor.
     */
    PanTiltRobotJoint(const PanTiltRobotJoint &src);

    /*!
     * \brief Destructor.
     */
    ~PanTiltRobotJoint();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltRobotJoint operator=(const PanTiltRobotJoint &rhs);

    /*!
     * \brief Get joint name.
     *
     * \return String.
     */
    std::string getJointName() const
    {
      return m_strName;
    }

    /*!
     * \brief Get master servo id.
     *
     * \return Returns servo id.
     */
    int getMasterServoId() const
    {
      return m_nMasterServoId;
    }

    /*!
     * \brief Get joint position and velocity limits.
     *
     * \param [out] fMinPos   Joint minimum position (radians).
     * \param [out] fMaxPos   Joint maximum position (radians).
     * \param [out] fMaxPos   Joint maximum velocity (radians/second).
     */
    void getJointLimits(double &fMinPos, double &fMaxPos, double &fMaxVel)
                                                                      const;

  protected:
    // (derived) specification
    std::string m_strName;            ///< joint name
    int         m_nMasterServoId;     ///< master servo id
    int         m_nSlaveServoId;      ///< linked slave servo id (if any)
    bool        m_bIsContinuous;      ///< servo should [not] be continuous mode
    int         m_nMasterServoDir;    ///< master servo normalized direction
    int         m_eJointType;         ///< joint type
    double      m_fGearRatio;         ///< joint gear ratio
    double      m_fTicksPerServoRad;  ///< encoder/odom. ticks per servo radian
    double      m_fTicksPerJointRad;  ///< encoder/odom. ticks per joint radian
    double      m_fMaxServoRadsPerSec;///< maximum servo radians per second
    double      m_fMaxJointRadsPerSec;///< maximum joint radians per second

    // discovered limits and positions
    int         m_nEncZeroPos;        ///< encoder value at zero position
    double      m_fMinPhyLimitRads;   ///< joint min physical limit (radians)
    double      m_fMaxPhyLimitRads;   ///< joint max physical limit (radians)
    int         m_nMinPhyLimitOd;     ///< joint min phys limit (odometer ticks)
    int         m_nMaxPhyLimitOd;     ///< joint max phys limit (odometer ticks)
    double      m_fMinSoftLimitRads;  ///< joint min soft limit (radians)
    double      m_fMaxSoftLimitRads;  ///< joint max soft limit (radians)
    int         m_nMinSoftLimitOd;    ///< joint min soft limit (odometer ticks)
    int         m_nMaxSoftLimitOd;    ///< joint max soft limit (odometer ticks)
    double      m_fCalibPosRads;      ///< joint calibrated position (radians)
    int         m_eLimitTypes;        ///< joint limit types
    uint_t      m_uTorqueLimit;       ///< joint servo(s) raw torque limit

    // state
    PanTiltOpState  m_eOpState;       ///< current operational state
    bool            m_bStopAtOptLimits; ///< do [not] stop at optical limits

    friend class PanTiltRobot;
    friend class PanTiltCalib;
  };


  // ---------------------------------------------------------------------------
  // Class PanTiltJointState
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Joint state class.
   *
   * This class encapsulates the current state of one joint.
   */
  class PanTiltJointState
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltJointState();

    /*!
     * \brief Copy constructor.
     */
    PanTiltJointState(const PanTiltJointState &src);

    /*!
     * \brief Destructor.
     */
    ~PanTiltJointState()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltJointState operator=(const PanTiltJointState &rhs);

    std::string m_strName;        ///< joint name
    PanTiltOpState  m_eOpState;   ///< joint operational state
    int         m_nMasterServoId; ///< master servo id
    int         m_nSlaveServoId;  ///< linked slave servo id (if any)
    double      m_fPosition;      ///< current joint position (radians)
    double      m_fVelocity;      ///< current joint velocity (% of max)
    double      m_fEffort;        ///< current joint effort (servo load)
    int         m_nOdPos;         ///< current master servo odometer pos (ticks)
    int         m_nEncPos;        ///< current master servo encoder pos (ticks)
    int         m_nSpeed;         ///< current master servo raw speed (unitless)
  };


  // ---------------------------------------------------------------------------
  // Class PanTiltJointStatePoint
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Joint state point class.
   *
   * This class encapsulates the current state of all joints in a kinematic
   * chain.
   *
   * A joint state point (J0, J1, ..., Jn-1) specifies the current dynamic
   * state of a set of joints (kinematic chain) Ji, i=0,n-1.
   */
  class PanTiltJointStatePoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltJointStatePoint()
    {
    }

    /*!
     * \brief Copy constructor.
     */
    PanTiltJointStatePoint(const PanTiltJointStatePoint &src)
    {
      m_strKinName = src.m_strKinName;
      m_jointState = src.m_jointState;
    }

    /*!
     * \brief Destructor.
     */
    ~PanTiltJointStatePoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltJointStatePoint operator=(const PanTiltJointStatePoint &rhs)
    {
      m_strKinName = rhs.m_strKinName;
      m_jointState = rhs.m_jointState;

      return *this;
    }

    /*!
     * \brief Get the kinematic chain name.
     *
     * \return String.
     */
    std::string getKinematicChainName()
    {
      return m_strKinName;
    }

    /*!
     * \brief Set the kinematic chain name.
     *
     * \param String.
     */
    void setKinematicChainName(std::string strKinName)
    {
      m_strKinName = strKinName;
    }

    /*!
     * \brief Get the number of joint states in joint state point.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_jointState.size();
    }

    /*!
     * \brief Test if joint is a member of the joint state.
     *
     * \param strJointName  Joint name.
     *
     * \return Returns true or false.
     */
    bool hasJoint(const std::string &strJointName);

    /*!
     * \brief Append joint state to end of joint state point.
     *
     * \param jointState    Joint state to append.
     */
    void append(const PanTiltJointState &jointState)
    {
      m_jointState.push_back(jointState);
    }

    /*!
     * \brief Subscript operator to get reference to joint point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param i   Index.
     *
     * \return Joint state.
     */
    PanTiltJointState &operator[](const size_t i)
    {
      return m_jointState[i];
    }

    /*!
     * \brief Key subscript operator to get reference to joint point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param strJointName  Joint name.
     *
     * \return Joint state.
     */
    PanTiltJointState &operator[](const std::string &strJointName);

    /*!
     * \brief Clear all state data.
     */
    void clear()
    {
      m_strKinName.clear();
      m_jointState.clear();
    }

  protected:
    std::string                     m_strKinName; ///< name of kinematic chain 
    std::vector<PanTiltJointState>  m_jointState; ///< vector of joint states
  };

} // namespace pan_tilt


#endif // _PT_JOINT_H
