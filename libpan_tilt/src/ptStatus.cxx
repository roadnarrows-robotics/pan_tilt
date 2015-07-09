////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptStatus.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt robot status classes implementations.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 * \author Daniel Packard   (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2015  RoadNarrows
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
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptStatus.h"

using namespace std;
using namespace pan_tilt;


// -----------------------------------------------------------------------------
// Class PanTiltServoHealth
// -----------------------------------------------------------------------------

PanTiltServoHealth::PanTiltServoHealth()
{
  m_nServoId      = DYNA_ID_NONE;
  m_fTemperature  = 0.0;
  m_fVoltage      = 0.0;
  m_uAlarms       = DYNA_ALARM_NONE;
}

PanTiltServoHealth::PanTiltServoHealth(const PanTiltServoHealth &src)
{
  m_nServoId      = src.m_nServoId;
  m_fTemperature  = src.m_fTemperature;
  m_fVoltage      = src.m_fVoltage;
  m_uAlarms       = src.m_uAlarms;
}

PanTiltServoHealth PanTiltServoHealth::operator=(const PanTiltServoHealth &rhs)
{
  m_nServoId      = rhs.m_nServoId;
  m_fTemperature  = rhs.m_fTemperature;
  m_fVoltage      = rhs.m_fVoltage;
  m_uAlarms       = rhs.m_uAlarms;

  return *this;
}


// -----------------------------------------------------------------------------
// Class PanTiltRobotStatus
// -----------------------------------------------------------------------------

PanTiltRobotStatus::PanTiltRobotStatus(const PanTiltRobotStatus &src)
{
  m_eRobotMode        = src.m_eRobotMode;
  m_eIsCalibrated     = src.m_eIsCalibrated;
  m_eIsEStopped       = src.m_eIsEStopped;
  m_eAreDrivesPowered = src.m_eAreDrivesPowered;
  m_eIsMotionPossible = src.m_eIsMotionPossible;
  m_eIsInMotion       = src.m_eIsInMotion;
  m_eIsInError        = src.m_eIsInError;
  m_nErrorCode        = src.m_nErrorCode;
  m_vecServoHealth    = src.m_vecServoHealth;
}

PanTiltRobotStatus PanTiltRobotStatus::operator=(const PanTiltRobotStatus &rhs)
{
  m_eRobotMode        = rhs.m_eRobotMode;
  m_eIsCalibrated     = rhs.m_eIsCalibrated;
  m_eIsEStopped       = rhs.m_eIsEStopped;
  m_eAreDrivesPowered = rhs.m_eAreDrivesPowered;
  m_eIsMotionPossible = rhs.m_eIsMotionPossible;
  m_eIsInMotion       = rhs.m_eIsInMotion;
  m_eIsInError        = rhs.m_eIsInError;
  m_nErrorCode        = rhs.m_nErrorCode;
  m_vecServoHealth    = rhs.m_vecServoHealth;

  return *this;
}

void PanTiltRobotStatus::clear()
{
  m_eRobotMode        = PanTiltRobotModeUnknown;
  m_eIsCalibrated     = PanTiltTriStateUnknown;
  m_eIsEStopped       = PanTiltTriStateUnknown;
  m_eAreDrivesPowered = PanTiltTriStateUnknown;
  m_eIsMotionPossible = PanTiltTriStateUnknown;
  m_eIsInMotion       = PanTiltTriStateUnknown;
  m_eIsInError        = PanTiltTriStateUnknown;
  m_nErrorCode        = PT_OK;
}
