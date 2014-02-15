////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptTraj.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Joint points and trajectory class interfaces.
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

#ifndef _PT_TRAJ_H
#define _PT_TRAJ_H

#include <sys/types.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "pan_tilt/pan_tilt.h"

namespace pan_tilt
{
  // ---------------------------------------------------------------------------
  // Class PanTiltJointTraj
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Joint trajectory class.
   *
   * A joint trajectory specifies the goal or current position, velocity, and
   * accelleration of one joint.
   */
  class PanTiltJointTraj
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltJointTraj()
    {
      m_fPosition     = 0.0;
      m_fVelocity     = 0.0;
      m_fAcceleration = 0.0;
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param strName       Joint name.
     * \param fPosition     Joint position.
     * \param fVelocity     Joint velocity.
     * \param fAcceleration Joint acceleration.
     */
    PanTiltJointTraj(const std::string &strName,
                  const double       fPosition,
                  const double       fVelocity,
                  const double       fAcceleration)
    {
      m_strName       = strName;
      m_fPosition     = fPosition;
      m_fVelocity     = fVelocity;
      m_fAcceleration = fAcceleration;
    }

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    PanTiltJointTraj(const PanTiltJointTraj &src)
    {
      m_strName       = src.m_strName;
      m_fPosition     = src.m_fPosition;
      m_fVelocity     = src.m_fVelocity;
      m_fAcceleration = src.m_fAcceleration;
    }

    /*!
     * \brief Destructor.
     */
    ~PanTiltJointTraj()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltJointTraj operator=(const PanTiltJointTraj &rhs)
    {
      m_strName       = rhs.m_strName;
      m_fPosition     = rhs.m_fPosition;
      m_fVelocity     = rhs.m_fVelocity;
      m_fAcceleration = rhs.m_fAcceleration;

      return *this;
    }

    /*!
     * \brief Get joint point data.
     *
     * \param [out] strName       Joint name.
     * \param [out] fPosition     Joint position.
     * \param [out] fVelocity     Joint velocity.
     * \param [out] fAcceleration Joint acceleration.
     */
    void get(std::string &strName,
             double      &fPosition,
             double      &fVelocity,
             double      &fAcceleration)
    {
      strName       = m_strName;
      fPosition     = m_fPosition;
      fVelocity     = m_fVelocity;
      fAcceleration = m_fAcceleration;
    }

  protected:
    std::string m_strName;        ///< joint name
    double      m_fPosition;      ///< joint position (radians)
    double      m_fVelocity;      ///< joint velocity (% of maximum)
    double      m_fAcceleration;  ///< joint acceleration (not used)
  };


  // ---------------------------------------------------------------------------
  // Class PanTiltJointTrajectoryPoint
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Joint trajectory point class.
   *
   * A joint trajectory point (T0, T1, ..., Tn-1) specifies the goal position,
   * velocity, and acceleration for a set of joints (kinematic chain)
   * Ti, i=0,n-1.
   */
  class PanTiltJointTrajectoryPoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltJointTrajectoryPoint()
    {
      m_uTimeFromStart = 0;
    }

    /*!
     * \brief Copy constructor.
     */
    PanTiltJointTrajectoryPoint(const PanTiltJointTrajectoryPoint &src)
    {
      m_uTimeFromStart  = src.m_uTimeFromStart;
      m_trajectory      = src.m_trajectory;
    }

    /*!
     * \brief Destructor.
     */
    ~PanTiltJointTrajectoryPoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltJointTrajectoryPoint operator=(
                                        const PanTiltJointTrajectoryPoint &rhs)
    {
      m_uTimeFromStart  = rhs.m_uTimeFromStart;
      m_trajectory      = rhs.m_trajectory;

      return *this;
    }

    /*!
     * \brief Get time from start.
     *
     * \todo TODO need to understand ROS meaning of this.
     *
     * \return Duration.
     */
    uint_t getTimeFromStart()
    {
      return m_uTimeFromStart;
    }

    /*!
     * \brief Set time from start.
     *
     * \todo TODO need to understand ROS meaning of this.
     *
     * \param uTimeFromStart    Duration.
     */
    void setTimeFromStart(uint_t uTimeFromStart)
    {
      m_uTimeFromStart = uTimeFromStart;
    }

    /*!
     * \brief Get the number of joint points in trajectory.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_trajectory.size();
    }

    /*!
     * \brief Append joint point to end of trajectory.
     *
     * \param strName       Joint name.
     * \param fPosition     Joint position.
     * \param fVelocity     Joint velocity.
     * \param fAcceleration Joint acceleration.
     */
    void append(const std::string &strName,
                double             fPosition,
                double             fVelocity,
                double             fAcceleration=0.0)
    {
      PanTiltJointTraj jointPoint(strName, fPosition, fVelocity, fAcceleration);
      m_trajectory.push_back(jointPoint);
    }

    /*!
     * \brief Subscript operator to get reference to joint point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param i   Index.
     *
     * \return Joint point.
     */
    PanTiltJointTraj &operator[](const size_t i)
    {
      return m_trajectory[i];
    }

    /*!
     * \brief Clear data.
     */
    void clear()
    {
      m_trajectory.clear();
      m_uTimeFromStart = 0;
    }

  protected:
    std::vector<PanTiltJointTraj>   m_trajectory;       ///< trajectory
    uint_t                      m_uTimeFromStart;   ///< duration
  };


  // ---------------------------------------------------------------------------
  // Class PanTiltJointTrajectoryFeedback
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Joint trajectory feedback class.
   *
   * The feedback provides informaation about the last issued desired trajectory
   * and the current trajectory.
   */
  class PanTiltJointTrajectoryFeedback
  {
  public:
    static const int  TRAJ_DESIRED  = 0;  ///< desired trajectory
    static const int  TRAJ_ACTUAL   = 1;  ///< actual trajectory
    static const int  TRAJ_NUMOF    = 2;  ///< number of trajectories

    /*!
     * \brief Default constructor.
     */
    PanTiltJointTrajectoryFeedback()
    {
    }

    /*!
     * \brief Copy constructor.
     */
    PanTiltJointTrajectoryFeedback(const PanTiltJointTrajectoryFeedback &src)
    {
      for(int i = 0; i< TRAJ_NUMOF; ++i)
      {
        m_trajectory[i] = src.m_trajectory[i];
      }
    }

    /*!
     * \brief Destructor.
     */
    ~PanTiltJointTrajectoryFeedback()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltJointTrajectoryFeedback operator=(
                                      const PanTiltJointTrajectoryFeedback &rhs)
    {
      for(int i = 0; i< TRAJ_NUMOF; ++i)
      {
        m_trajectory[i] = rhs.m_trajectory[i];
      }
      return *this;
    }

    /*!
     * \brief Get time from start of desired trajectory.
     *
     * \return Duration.
     */
    uint_t getTimeFromStart()
    {
      return m_trajectory[TRAJ_DESIRED].getTimeFromStart();
    }

    /*!
     * \brief Get the number of joint points in trajectory.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_trajectory[TRAJ_DESIRED].getNumPoints();
    }

    /*!
     * \brief Subscript operator to get reference to given trajectory.
     *
     * \param i   Trajectory point Index.
     *
     * \return Trajectory.
     */
    PanTiltJointTrajectoryPoint &operator[](const size_t i)
    {
      return i < TRAJ_NUMOF? m_trajectory[i]: m_trajectory[TRAJ_DESIRED];
    }

    /*!
     * \brief Clear data.
     */
    void clear()
    {
      for(int i = 0; i < TRAJ_NUMOF; ++i)
      {
        m_trajectory[i].clear();
      }
    }

  protected:
    PanTiltJointTrajectoryPoint   m_trajectory[TRAJ_NUMOF];
  };
} // namespace pan_tilt


#endif // _PT_TRAJ_H
