////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptCalib.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltCalib - Pan-Tilt calibration class implementation.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2015  RoadNarrows
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

#include "unistd.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaServo.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptXmlCalib.h"
#include "pan_tilt/ptJoint.h"
#include "pan_tilt/ptCalib.h"
#include "pan_tilt/ptRobot.h"

using namespace std;
using namespace pan_tilt;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// debug switch
#undef  CALIB_DEBUG_ENABLE

#ifdef CALIB_DEBUG_ENABLE

#define CALIB_DBG(servoid, fmt, ...) \
  do \
  { \
    fprintf(stderr, "DBG: %s(): Servo: %d: " fmt, \
      LOGFUNCNAME, servoid, ##__VA_ARGS__); \
    fflush(stderr); \
  } while(0)

#else

#define CALIB_DBG(servoid, fmt, ...)

#endif // CALIB_DEBUG_ENABLE


/*!
 * \brief Convenience macro for trying to get a servo object from dynamixel
 * chain.
 *
 * Failure is considered a software bug since the chain has already be 
 * verified.
 *
 * Only works locally.
 *
 * \param [in] nServoId   Servo id.
 * \param [out] pServo    Pointer to associated servo object.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define PT_TRY_GET_SERVO(nServoId, pServo) \
  do \
  { \
    if( (pServo = m_robot.m_pDynaChain->GetServo(nServoId)) == NULL ) \
    { \
      LOGERROR("BUG: Servo %d: Cannot find in dynamixel chain.", nServoId); \
      return -PT_ECODE_INTERNAL; \
    } \
  } while(0)

//
// Fixed calibration data.
//
static const char *CalibOrder[] = { "pan", "tilt" };  ///< calibration order
static int         TuneCalSpeed = 75;                 ///< calibration speed
static int         TuneBackoff  = 15;                 ///< backoff pos (ticks)


int PanTiltCalib::calibrateFromParams(const PanTiltCalibParams &params)
{
  PanTiltRobotJoint  *pJoint;
  int                 i;
  int                 rc;

  if( !params.m_bIsSpecified )
  {
    LOGERROR("Calibration parameters not fully specified.");
    return -PT_ECODE_BAD_VAL;
  }

  for(i=0, rc=PT_OK; (i<arraysize(CalibOrder)) && (rc >= 0); ++i)
  {
    if( (pJoint = m_robot.getJoint(CalibOrder[i])) == NULL )
    {
      LOGERROR("BUG: Joint %s: Cannot find.", CalibOrder[i]);
      rc = -PT_ECODE_INTERNAL;
    }
    else
    {
      rc = calibrateJointFromParams(*pJoint, params);
    }

    pJoint->m_eOpState =  rc == PT_OK? PanTiltOpStateCalibrated:
                                       PanTiltOpStateUncalibrated;
  }

  return rc;
}

int PanTiltCalib::calibrateJointFromParams(PanTiltRobotJoint        &joint,
                                           const PanTiltCalibParams &params)
{
  int         nServoId;     // servo id
  bool        bIsReverse;   // odometer is [not] reversed
  int         nSign;        // encoder reverse/normal sign
  DynaServo  *pServo;       // joint's master servo
  int         nEncZeroPos;  // encoder zero position
  int         nOdMinPos;    // encoder minimum odometer position
  int         nOdMaxPos;    // encoder maximum odometer position
  int         nDeltaPos;    // working delta position
  int         nOdGoalPos;   // working goal position
  int         nOdCurPos;    // working current position
  int         nServoEncMin; // servo encoder minimum
  int         nServoEncMax; // servo encoder maximum

  nServoId    = joint.m_nMasterServoId;
  bIsReverse  = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;
  nSign       = bIsReverse? -1: 1;

  // dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  if( joint.m_strName == "pan" )
  {
    nEncZeroPos = params.m_nPanEncZero;
    nOdMinPos   = params.m_nPanOdMin;
    nOdMaxPos   = params.m_nPanOdMax;
  }
  else if( joint.m_strName == "tilt" )
  {
    nEncZeroPos = params.m_nTiltEncZero;
    nOdMinPos   = params.m_nTiltOdMin;
    nOdMaxPos   = params.m_nTiltOdMax;
  }
  else
  {
    LOGERROR("BUG: Joint %s: Unknown.", joint.m_strName.c_str());
    return -PT_ECODE_INTERNAL;
  }

  //
  // Validate parameters.
  //
  nServoEncMin = pServo->GetSpecification().m_uRawPosMin;
  nServoEncMax = pServo->GetSpecification().m_uRawPosMax;

  if( (nEncZeroPos < nServoEncMin) || (nEncZeroPos > nServoEncMax) )
  {
    LOGERROR("Joint %s (servo=%d): "
        "%d: Zero position encoder value out-of-range.",
      joint.m_strName.c_str(), nServoId, nEncZeroPos);
    return -PT_ECODE_BAD_VAL;
  }

  switch( pServo->GetServoMode() )
  {
    case DYNA_MODE_CONTINUOUS:
      break;
    case DYNA_MODE_SERVO:
    default:
      if( iabs(nOdMaxPos - nOdMinPos) > (nServoEncMax - nServoEncMin) )
      {
        LOGERROR("Joint %s (servo=%d): "
          "%d: (%d,%d): Joint (min,max) range is out of servo movement range.",
          joint.m_strName.c_str(), nServoId, nOdMinPos, nOdMaxPos);
        return -PT_ECODE_BAD_VAL;
      }
      break;
  }

  // current zero odometer position
  nDeltaPos = pServo->OdometerToEncoder(0);

  // new zero position
  nOdGoalPos = nEncZeroPos - nSign * nDeltaPos;

  //
  // Move to new zero point position.
  //
  LOGDIAG2("Joint %s (servo=%d): "
      "Move to the new zero point at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdGoalPos));

  moveWait(pServo, nOdGoalPos, TuneCalSpeed);

  nOdCurPos = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdCurPos));

  // 
  // Reset odometer to new zero degree position.
  //
  m_robot.resetOdometerForServo(nServoId, bIsReverse);

  LOGDIAG2("Joint %s (servo=%d): Odometer reset.",
        joint.m_strName.c_str(), nServoId);

  joint.m_nEncZeroPos       = pServo->OdometerToEncoder(0);
  joint.m_nMinPhyLimitOd    = nOdMinPos;
  joint.m_nMaxPhyLimitOd    = nOdMaxPos;
  joint.m_fMinPhyLimitRads  = (double)joint.m_nMinPhyLimitOd / 
                                      joint.m_fTicksPerJointRad;
  joint.m_fMaxPhyLimitRads  = (double)joint.m_nMaxPhyLimitOd / 
                                      joint.m_fTicksPerJointRad;
  joint.m_nMinSoftLimitOd   = joint.m_nMinPhyLimitOd + TuneBackoff;
  joint.m_nMaxSoftLimitOd   = joint.m_nMaxPhyLimitOd - TuneBackoff;

  joint.m_fMinSoftLimitRads = (double)joint.m_nMinSoftLimitOd / 
                                      joint.m_fTicksPerJointRad;
  joint.m_fMaxSoftLimitRads = (double)joint.m_nMaxSoftLimitOd / 
                                      joint.m_fTicksPerJointRad;
  
  //
  // Calibrated.
  //
  LOGDIAG2("Joint %s (servo=%d): Calibrated from parameters:\n"
          "  Physical limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)]\n"
          "      Soft limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)].",
      joint.m_strName.c_str(), nServoId,
      radToDeg(joint.m_fMinPhyLimitRads),  joint.m_nMinPhyLimitOd,
      radToDeg(joint.m_fMaxPhyLimitRads),  joint.m_nMaxPhyLimitOd,
      radToDeg(joint.m_fMinSoftLimitRads), joint.m_nMinSoftLimitOd,
      radToDeg(joint.m_fMaxSoftLimitRads), joint.m_nMaxSoftLimitOd);

  return PT_OK;
}

int PanTiltCalib::calibrate()
{
  PanTiltRobot::MapRobotJoints::iterator  pos;

  PanTiltRobotJoint *pJoint;
  int               nServoId;
  size_t            i;
  int               rc;

  for(i=0, rc=PT_OK; (i<arraysize(CalibOrder)) && (rc >= 0); ++i)
  {
    nServoId = m_robot.getMasterServoId(CalibOrder[i]);

    // no joint in this product version with associated master servo id
    if( (pos = m_robot.m_kin.find(nServoId)) == m_robot.m_kin.end() )
    {
      continue;
    }

    pJoint = &(pos->second);

    // already calibrated
    if( pJoint->m_eOpState == PanTiltOpStateCalibrated )
    {
      continue;
    }

    pJoint->m_eOpState  = PanTiltOpStateCalibrating;

    // physical limits
    rc = calibrateJointByTorqueLimits(*pJoint);

    pJoint->m_eOpState =  rc == PT_OK? PanTiltOpStateCalibrated:
                                       PanTiltOpStateUncalibrated;
  }

  return rc;
}

int PanTiltCalib::calibrateJointByTorqueLimits(PanTiltRobotJoint &joint)
{
  int         nServoId;         // servo id
  DynaServo  *pServo;           // joint's master servo
  bool        bIsReverse;       // odometer is [not] reversed
  int         nOdStartPos;      // user's starting position 
  int         nOdGoalPos;       // working goal position
  int         nOdCurPos;        // working current position
  int         nOdZeroPos;       // calibrated zero position
  int         nNewMinLimit;     // new odometer minimum limit
  int         nNewMaxLimit;     // new odometer maximum limit
  int         nOdRange;         // joint range

  nServoId      = joint.m_nMasterServoId;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;

  LOGDIAG2("Joint %s (servo=%d): Calibrating by torque limits.",
      joint.m_strName.c_str(), nServoId);

  // dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  nOdStartPos = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): "
    "Starting at %.2lf uncalibrated degrees (od=%d).",
    joint.m_strName.c_str(), nServoId,
    degrees(joint, nOdStartPos), nOdStartPos);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find minimum joint limit.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  LOGDIAG2("Joint %s (servo=%d): "
           "Determining minimum joint angle by torque limit.",
      joint.m_strName.c_str(), nServoId);

  //
  // A continuous mode servo must have its joint minimum limit within 360
  // degreees of joint rotation from the current position. Go to this endpoint
  // until the torque limit is found.
  //
  if( joint.m_eJointType == PanTiltJointTypeContinuous )
  {
    nOdGoalPos = M_TAU * -joint.m_fTicksPerJointRad;
  }

  //
  // A servo mode servo has encoder endpoints that are known and finite. Go to
  // the appropriate endpoint until the torque limit is found.
  //
  else
  {
    if( (nOdGoalPos = pServo->CalcOdometerAtEncMin()) > 0 )
    {
      nOdGoalPos = pServo->CalcOdometerAtEncMax();
    }
  }

  LOGDIAG2("Joint %s (servo=%d): Move to minimum >= %.2lf degrees (od=%d).",
    joint.m_strName.c_str(), nServoId, degrees(joint, nOdGoalPos), nOdGoalPos);

  moveWait(pServo, nOdGoalPos, TuneCalSpeed);

  nNewMinLimit = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): "
      "Minimum reached at %.2lf uncalibrated degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nNewMinLimit));

  //
  // Off load servo by moving back to starting calibration position.
  //
  LOGDIAG2("Joint %s (servo=%d): "
      "Move back to start at %.2lf uncalibrated degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdStartPos));

  moveWait(pServo, nOdStartPos, TuneCalSpeed);

  nOdCurPos = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdCurPos));


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find maximum limit.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  LOGDIAG2("Joint %s (servo=%d): "
           "Determining maximum joint angle by torque limit.",
      joint.m_strName.c_str(), nServoId);

  //
  // A continuous mode servo must have its joint minimum limit within 360
  // degreees of joint rotation from the current position. Go to this endpoint
  // until the torque limit is found.
  //
  if( joint.m_eJointType == PanTiltJointTypeContinuous )
  {
    nOdGoalPos = M_TAU * joint.m_fTicksPerJointRad;
  }

  //
  // A servo mode servo has encoder endpoints that are known and finite. Go to
  // the appropriate endpoint until the torque limit is found.
  //
  else
  {
    if( (nOdGoalPos = pServo->CalcOdometerAtEncMin()) < 0 )
    {
      nOdGoalPos = pServo->CalcOdometerAtEncMax();
    }
  }

  LOGDIAG2("Joint %s (servo=%d): Move to maximum <= %.2lf degrees (od=%d).",
    joint.m_strName.c_str(), nServoId, degrees(joint, nOdGoalPos), nOdGoalPos);

  moveWait(pServo, nOdGoalPos, TuneCalSpeed);

  nNewMaxLimit = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): "
      "Minimum reached at %.2lf uncalibrated degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nNewMaxLimit));

  //
  // Off load servo by moving back to starting calibration position
  //
  LOGDIAG2("Joint %s (servo=%d): "
      "Move back to start at %.2lf uncalibrated degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdStartPos));

  moveWait(pServo, nOdStartPos, TuneCalSpeed);

  nOdCurPos = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdCurPos));


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Set zero point and limits
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Calculate new zero point at center of range of motion.
  //
  nOdRange   = (nNewMaxLimit - nNewMinLimit) / 2; // half range
  nOdZeroPos = (nNewMaxLimit + nNewMinLimit) / 2; // uncalib center position

  LOGDIAG2("Joint %s (servo=%d): "
      "Move to the new zero point at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdZeroPos));

  //
  // Move to new zero point position.
  //
  moveWait(pServo, nOdZeroPos, TuneCalSpeed);

  nOdCurPos = pServo->GetOdometer();

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdCurPos));

  // 
  // Reset odometer to new zero degree position.
  //
  m_robot.resetOdometerForServo(nServoId, bIsReverse);

  LOGDIAG2("Joint %s (servo=%d): Odometer reset.",
        joint.m_strName.c_str(), nServoId);

  //
  // Adjust limits and positions to adjusted zero degree point.
  //
  joint.m_nEncZeroPos       = pServo->OdometerToEncoder(0);
  joint.m_nMinPhyLimitOd    = -nOdRange;
  joint.m_nMaxPhyLimitOd    = nOdRange;
  joint.m_fMinPhyLimitRads  = (double)joint.m_nMinPhyLimitOd / 
                                      joint.m_fTicksPerJointRad;
  joint.m_fMaxPhyLimitRads  = (double)joint.m_nMaxPhyLimitOd / 
                                      joint.m_fTicksPerJointRad;
  joint.m_nMinSoftLimitOd   = joint.m_nMinPhyLimitOd + TuneBackoff;
  joint.m_nMaxSoftLimitOd   = joint.m_nMaxPhyLimitOd - TuneBackoff;

  joint.m_fMinSoftLimitRads = (double)joint.m_nMinSoftLimitOd / 
                                      joint.m_fTicksPerJointRad;
  joint.m_fMaxSoftLimitRads = (double)joint.m_nMaxSoftLimitOd / 
                                      joint.m_fTicksPerJointRad;

  //
  // Finally move to ideal calibration position if not zero.
  //
  if( joint.m_fCalibPosRads != 0.0 )
  { 
    LOGDIAG2("Joint %s (servo=%d): Move to calibrated zero pt=%.2lf degrees.",
        joint.m_strName.c_str(), nServoId, radToDeg(joint.m_fCalibPosRads));

    nOdGoalPos = (int)(joint.m_fCalibPosRads * joint.m_fTicksPerJointRad);

    moveWait(pServo, nOdGoalPos, TuneCalSpeed);

    nOdCurPos = pServo->GetOdometer();

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
      joint.m_strName.c_str(), nServoId, degrees(joint, nOdCurPos));
  }

  //
  // Calibrated.
  //
  LOGDIAG2("Joint %s (servo=%d): Calibrated by torque:\n"
          "  Physical limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)]\n"
          "      Soft limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)].",
      joint.m_strName.c_str(), nServoId,
      radToDeg(joint.m_fMinPhyLimitRads),  joint.m_nMinPhyLimitOd,
      radToDeg(joint.m_fMaxPhyLimitRads),  joint.m_nMaxPhyLimitOd,
      radToDeg(joint.m_fMinSoftLimitRads), joint.m_nMinSoftLimitOd,
      radToDeg(joint.m_fMaxSoftLimitRads), joint.m_nMaxSoftLimitOd);

  return PT_OK;
}

int PanTiltCalib::moveWait(DynaServo *pServo, int nOdGoalPos, int nSpeed)
{
  static uint_t TuneWaituSec  = 300000;
  static int    TuneDeltaErr  = 5;

  int   nOdBefore;
  int   nOdAfter;
  bool  bMoving;

  nSpeed *= pServo->CalcSpeedDir(nOdGoalPos);

  // start move
  pServo->MoveAtSpeedTo(nSpeed, nOdGoalPos);

  CALIB_DBG(pServo->GetServoId(), "moveWait: move at speed=%d to %d\n",
                nSpeed, nOdGoalPos);

  // wait until move is completed
  do
  {
    nOdBefore = pServo->GetOdometer();
    usleep(TuneWaituSec);   // pthread cancelation point
    nOdAfter = pServo->GetOdometer();

    CALIB_DBG(pServo->GetServoId(), "moveWait: curspeed=%d, curpos=%d\n",
          pServo->GetCurSpeed(), nOdAfter);

    pServo->ReadIsMoving(&bMoving);
  }
  while( bMoving );
  //while( iabs(nOdBefore - nOdAfter) > TuneDeltaErr );

  return pServo->GetOdometer();
}

double PanTiltCalib::degrees(PanTiltRobotJoint &joint, int nOdPos)
{
  return m_robot.odPosToJointPosition(joint, nOdPos, units_degrees);
}
