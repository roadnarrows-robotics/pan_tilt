////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptStatus.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltateros Robot Status classes interface.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _PT_STATUS_H
#define _PT_STATUS_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#include "pan_tilt/pan_tilt.h"

namespace pan_tilt
{
  // ---------------------------------------------------------------------------
  // Class PanTiltServoHealth
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot servo health.
   */
  class PanTiltServoHealth
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltServoHealth();

    /*!
     * \brief Copy constructor.
     */
    PanTiltServoHealth(const PanTiltServoHealth &src);

    /*!
     * \brief Destructor.
     */
    ~PanTiltServoHealth()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltServoHealth operator=(const PanTiltServoHealth &rhs);

    int     m_nServoId;       ///< servo id
    float   m_fTemperature;   ///< servo temperature (Celsius)
    float   m_fVoltage;       ///< servo voltage (volts)
    uint_t  m_uAlarms;        ///< servo alarms
  };


  // ---------------------------------------------------------------------------
  // Class PanTiltRobotStatus
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot overall status.
   */
  class PanTiltRobotStatus
  {
  public:
    /*! vector of health */
    typedef std::vector<PanTiltServoHealth>   VecHealth;

    /*!
     * \brief Default constructor.
     */
    PanTiltRobotStatus()
    {
      clear();
    }

    /*!
     * \brief Copy constructor.
     */
    PanTiltRobotStatus(const PanTiltRobotStatus &src);

    /*!
     * \brief Destructor.
     */
    ~PanTiltRobotStatus()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    PanTiltRobotStatus operator=(const PanTiltRobotStatus &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    PanTiltRobotMode m_eRobotMode;        ///< robot operating mode
    PanTiltTriState m_eIsCalibrated;      ///< robot is [not] calibrated
    PanTiltTriState m_eIsEStopped;        ///< robot is [not] emergency stopped
    PanTiltTriState m_eAreDrivesPowered;  ///< servos are [not] powered
    PanTiltTriState m_eIsMotionPossible;  ///< motion is [not] possible
    PanTiltTriState m_eIsInMotion;        ///< robot is [not] moving
    PanTiltTriState m_eIsInError;         ///< robot is [not] in error condition
    int             m_nErrorCode;         ///< pan_tilt error code
    VecHealth       m_vecServoHealth;     ///< servos' health
  };

} // namespace pan_tilt

#endif // _PT_STATUS_H
