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
 * \brief PanTiltCalib - Pan-Tilt calibration class
 * implementation.
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaServo.h"

#include "pan_tilt/pan_tilt.h"
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

/*!
 * \brief Calibration order by master servo id.
 */
static int CalibOrder[] =
{
  PanTiltServoIdTilt,
  PanTiltServoIdPan
};

int PanTiltCalib::calibrate()
{
  PanTiltRobot::MapRobotJoints::iterator  pos;

  PanTiltRobotJoint  *pJoint;
  int                 nServoId;
  size_t            i;
  int               rc;

  for(i=0, rc=PT_OK; (i<arraysize(CalibOrder)) && (rc >= 0); ++i)
  {
    nServoId = CalibOrder[i];

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
  static int    TuneCalSpeed  = 75;     // calibration speed
  static int    TuneBackoff   = 2;      // backoff position (ticks)

  int         nServoId;         // servo id
  DynaServo  *pServo;           // joint's master servo
  bool        bIsReverse;       // odometer is [not] reversed
  int         nOdStartPos;      // user's starting position 
  int         nOdGoalPos;       // working goal position
  int         nOldMinLimit;     // original odometer minimum limit
  int         nOldMaxLimit;     // original odometer maximum limit
  int         nNewMinLimit;     // new odometer minimum limit
  int         nNewMaxLimit;     // new odometer maximum limit
  int         nDeltaMin;        // delta odometer minimums
  int         nDeltaMax;        // delta odometer maximums
  int         nDeltaAvg;        // delta average

  nServoId      = joint.m_nMasterServoId;
  nOldMinLimit  = joint.m_nMinPhyLimitOd;
  nOldMaxLimit  = joint.m_nMaxPhyLimitOd;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;

  // dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  nOdStartPos = pServo->GetOdometer();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find minimum joint limit.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  LOGDIAG3("Joint %s (servo=%d): "
           "Determining minimum joint limit by torque limit.",
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

  //LOGDIAG3("MOVE TO MINIMUM LIMIT.");
  moveWait(pServo, nOdGoalPos, TuneCalSpeed);

  nNewMinLimit = pServo->GetOdometer();

  //
  // Off load servo by moving back to starting calibration position.
  //
  //LOGDIAG3("MOVE BACK TO START.");
  moveWait(pServo, nOdStartPos, TuneCalSpeed);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find maximum limit.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  LOGDIAG3("Joint %s (servo=%d): "
           "Determining maximum joint limit by torque limit.",
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

  //LOGDIAG3("MOVE TO MAXIMUM LIMIT.");
  moveWait(pServo, nOdGoalPos, TuneCalSpeed);

  nNewMaxLimit = pServo->GetOdometer();

  //
  // Off load servo by moving back to starting calibration position
  //
  //LOGDIAG3("MOVE BACK TO START AGAIN.");
  moveWait(pServo, nOdStartPos, TuneCalSpeed);

  //
  // Calculate best new zero degree point.
  //
  nDeltaMin = nNewMinLimit - nOldMinLimit;
  nDeltaMax = nNewMaxLimit - nOldMaxLimit;
  nDeltaAvg = (nDeltaMin + nDeltaMax) / 2;

  //
  // Move to new zero point position.
  //
  //LOGDIAG3("MOVE TO NEW ZERO=%d.", nOdStartPos+nDeltaAvg);
  moveWait(pServo, nOdStartPos+nDeltaAvg, TuneCalSpeed);

  // 
  // Reset odometer to new zero degree position.
  //
  m_robot.resetOdometerForServo(nServoId, bIsReverse);

  //
  // Adjust limits and positions to adjusted zero degree point.
  //
  joint.m_nMinPhyLimitOd    = nNewMinLimit - nDeltaAvg;
  joint.m_nMaxPhyLimitOd    = nNewMaxLimit - nDeltaAvg;

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
  // Finally move to ideal calibration position.
  //
  if( joint.m_fCalibPosRads != 0.0 )
  { 
    nOdGoalPos = (int)(joint.m_fCalibPosRads * joint.m_fTicksPerJointRad);
    //LOGDIAG3("MOVE TO CALIB ZERO PT=%d.", nOdGoalPos);
    moveWait(pServo, nOdGoalPos, TuneCalSpeed);
  }

  //
  // Calibrated.
  //
  LOGDIAG3("Joint %s (servo=%d): Calibrated:\n"
          "  Torque [min, max] limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)].",
      joint.m_strName.c_str(), nServoId,
      radToDeg(joint.m_fMinPhyLimitRads), joint.m_nMinPhyLimitOd,
      radToDeg(joint.m_fMaxPhyLimitRads), joint.m_nMaxPhyLimitOd);

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
