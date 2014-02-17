////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptRobot.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltRobot - the pan-tilt robot class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"
#include "Dynamixel/DynaError.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"
#include "pan_tilt/ptSpec.h"
#include "pan_tilt/ptDesc.h"
#include "pan_tilt/ptJoint.h"
#include "pan_tilt/ptCalib.h"
#include "pan_tilt/ptTraj.h"
#include "pan_tilt/ptStatus.h"
#include "pan_tilt/ptRobot.h"

using namespace std;
using namespace pan_tilt;

/*!
 * \brief Convenience macro for trying to get a servo object from dynamixel
 * chain.
 *
 * Failure is considered a software bug since the chain has already be 
 * verified.
 *
 * Only works in PanTiltRobot methods.
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
    if( (pServo = m_pDynaChain->GetServo(nServoId)) == NULL ) \
    { \
      LOGERROR("BUG: Servo %d: Cannot find in dynamixel chain.", nServoId); \
      return -PT_ECODE_INTERNAL; \
    } \
  } while(0)


/*!
 * \brief Test for connection.
 *
 * Only works in PanTiltRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define PT_TRY_CONN() \
  do \
  { \
    if( !isConnected() ) \
    { \
      LOGERROR("Robot is not connected."); \
      return -PT_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for calibration.
 *
 * Only works in PanTiltRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define PT_TRY_CALIB() \
  do \
  { \
    if( m_eOpState != PanTiltOpStateCalibrated ) \
    { \
      LOGERROR("Robot is not calibrated."); \
      return -PT_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for not estop.
 *
 * Only works in PanTiltRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define PT_TRY_NOT_ESTOP() \
  do \
  { \
    if( m_bIsEStopped ) \
    { \
      LOGERROR("Robot is emergency stopped."); \
      return -PT_ECODE_NO_EXEC; \
    } \
  } while(0)


// -----------------------------------------------------------------------------
// Class PanTiltRobot
// -----------------------------------------------------------------------------

PanTiltRobot::PanTiltRobot()
{
  // state
  m_eRobotMode        = PanTiltRobotModeAuto;
  m_eOpState          = PanTiltOpStateUncalibrated;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;
  m_bAreServosPowered = false;

  // dynamixel i/f
  m_pDynaComm         = NULL;
  m_pDynaChain        = NULL;
  m_pDynaBgThread     = NULL;

  // asynchronous task control
  m_eAsyncTaskState   = PanTiltAsyncTaskStateIdle;
  m_rcAsyncTask       = PT_OK;
  m_eAsyncTaskId      = AsyncTaskIdNone;
  m_pAsyncTaskArg     = NULL;
}

PanTiltRobot::~PanTiltRobot()
{
  disconnect();
}

int PanTiltRobot::connect(const string &strSerDevName, int nBaudRate)
{
  int   rc;   // return code

  // open serial device to dynamixel bus
  m_pDynaComm = DynaComm::New(strSerDevName.c_str(), nBaudRate);

  if( m_pDynaComm == NULL )
  {
    LOGERROR("Failed to create dynamixel interface on '%s'@%d.",
        strSerDevName.c_str(), nBaudRate);
    return -PT_ECODE_DYNA;
  }

  // create dynamixel bus chain
  m_pDynaChain = new DynaChain(*m_pDynaComm);

  // scan pan-tilt hardware and verify
  if( (rc = scan()) < 0 )
  {
    LOGERROR("Pan-Tilt hardware scan failed.");
    disconnect();
    return rc;
  }

  // convert fixed product specifications to operational parameters
  if( (rc = convertSpec()) < 0 )
  {
    LOGERROR("Failed to convert product specifications to "
             "operations parameters.");
    disconnect();
    return rc;
  }

  //
  // Create dynamixel background virtual servo thread.
  //  Execution Cylcle:   10Hz == 100000 usec
  //  Position Tolerance: plus/minus 0.2 degrees.
  //
  m_pDynaBgThread = new DynaBgThread(100000, 0.2);

  // register dynamixel chain with thread
  m_pDynaBgThread->RegisterChainAgent(m_pDynaChain);

  // run the thread
  m_pDynaBgThread->Run();
  
  // give a little time to bg thread
  usleep(1000);

  LOGDIAG1("Connected to Pan-Tilt.");

  //
  // DBG DANGER, WILL ROBINSON
  //
  // Uncomment the next line to circumvent calibration step. Make sure arm
  // is at the zero point.
  // 
  //fauxcalibrate();
  //
  // DBG DANGER, WILL ROBINSON
  //

  return PT_OK;
}

int PanTiltRobot::disconnect()
{
  // stop and destroy background virtual servo thread
  if( m_pDynaBgThread != NULL )
  {
    m_pDynaBgThread->Stop();
    m_pDynaBgThread->UnregisterUserCallback();
    m_pDynaBgThread->UnregisterAgent();
    delete m_pDynaBgThread;
    m_pDynaBgThread = NULL;
  }

  // delete dynamixel chain
  if( m_pDynaChain != NULL )
  {
    m_pDynaChain->EStop();
    delete m_pDynaChain;
    m_pDynaChain = NULL;
  }

  // close connection and delete dynamixel communication object
  if( m_pDynaComm != NULL )
  {
    if( m_pDynaComm->IsOpen() )
    {
      m_pDynaComm->Close();
    }
    delete m_pDynaComm;
    m_pDynaComm = NULL;
  }

  // reset state flags
  m_eOpState = PanTiltOpStateUncalibrated;

  LOGDIAG1("Disconnected from Pan-Tilt.");
}

int PanTiltRobot::calibrate(bool bForceRecalib)
{
  PanTiltCalib    calib(*this);
  int             rc;

  PT_TRY_CONN();

  // lock the arm
  freeze();

  m_eOpState = PanTiltOpStateUncalibrated;

  // configure arm for calibration
  if( (rc = configForCalibration(bForceRecalib)) < 0 )
  {
    LOGERROR("Failed to configure Pan-Tilt for calibration.");
    return rc;
  }

  m_eOpState = PanTiltOpStateCalibrating;

  // now physically calibrate the arm
  if( (rc = calib.calibrate()) < 0 )
  {
    LOGERROR("Failed to detect Pan-Tilt limits.");
    m_eOpState = PanTiltOpStateUncalibrated;
    return rc;
  }

  // reconfigure for normal operation
  if( (rc = configForOperation()) < 0 )
  {
    LOGERROR("Failed to configure Pan-Tilt for operation.");
    m_eOpState = PanTiltOpStateUncalibrated;
    return rc;
  }

  // synchronize robot operation state to collective joint operational states
  m_eOpState = detRobotOpState();

  // arm is calibrated
  gotoZeroPtPos();

  LOGDIAG1("Pan-Tilt calibrated.");

  return PT_OK;
}

int PanTiltRobot::gotoZeroPtPos()
{
  PanTiltJointTrajectoryPoint trajPoint;
  MapRobotJoints::iterator    iter;

  int                 nMasterServoId;
  PanTiltRobotJoint  *pJoint;
  int                 rc;

  PT_TRY_CONN();
  PT_TRY_CALIB();
  PT_TRY_NOT_ESTOP();

  //
  // Build trajoectory point.
  //
  for(iter = m_kin.begin(); iter != m_kin.end(); ++iter)
  {
    nMasterServoId  = iter->first;
    pJoint          = &(iter->second);

    switch( nMasterServoId )
    {
      case PanTiltServoIdPan:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads, 5.0);
        break;
      case PanTiltServoIdTilt:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads, 5.0);
        break;
    }
  }

  if( (rc = move(trajPoint)) < 0 )
  {
    LOGERROR("Move to pre-defined calibrated zero point position failed.");
  }
  else
  {
    LOGDIAG3("Pan-Tilt at zero point position.");
  }

  return rc;
}

int PanTiltRobot::estop()
{
  PT_TRY_CONN();

  m_pDynaChain->EStop();

  m_bIsEStopped       = true;
  m_bAreServosPowered = false;

  m_lastTraj.clear();

  m_bAlarmState = true;

  LOGDIAG3("Pan-Tilt emergency stopped.");

  return PT_OK;
}

int PanTiltRobot::freeze()
{
  PT_TRY_CONN();

  m_bAreServosPowered = true;

  m_lastTraj.clear();

  m_pDynaChain->Freeze();

  LOGDIAG3("Pan-Tilt frozen at current position.");

  return PT_OK;
}

int PanTiltRobot::release()
{
  PT_TRY_CONN();

  m_bAreServosPowered = false;

  m_lastTraj.clear();

  m_pDynaChain->Release();

  LOGDIAG3("Pan-Tilt servo drives released.");

  return PT_OK;
}

int PanTiltRobot::clearAlarms()
{
  int             iter;       // servo iterator
  int             nServoId;   // servo id
  DynaServo      *pServo;     // servo

  PT_TRY_CONN();

  //
  // Try to clear alarms for all servos.
  //
  for(nServoId = m_pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId = m_pDynaChain->IterNext(&iter))
  {
    pServo = m_pDynaChain->GetServo(nServoId);

    // stop
    pServo->Stop();

    // over torque alarm zero's torque limit - reload from EEPROM
    pServo->ReloadMaxTorqueLimit();
  }

  //
  // Clear pan-tilt alarm led to provide visual feedback to the user. The
  // background check may relight the alarm led if alarms still persist.
  //
  m_bAlarmState = false;

  return PT_OK;
}

int PanTiltRobot::move(PanTiltJointTrajectoryPoint &trajectoryPoint)
{
  static int      TuneDeltaPos  = 5;  // current-goal identical position delta

  PanTiltRobotJoint    *pJoint;          // robotic joint
  string                strName;          // joint name
  double                fPosition;        // goal position
  double                fVelocity;        // goal velocity
  double                fAcceleration;    // goal acceleration
  int                   nOdPos;           // converted odometer position
  int                   nRawSpeed;        // converted raw speed
  DynaServo            *pServo;           // master servo
  DynaSpeedPosTuple_T   tupSpeedPos[DYNA_ID_NUMOF]; // raw tuple array
  int                   nNumPts;          // number of tuple points
  uint_t                i;                // working index
  int                   rc;               // return code

  PT_TRY_CONN();
  PT_TRY_CALIB();
  PT_TRY_NOT_ESTOP();

  // automatically power up servos if not powered
  if( !m_bAreServosPowered )
  {
    freeze();
  }

  //
  // Convert trajectory point to tuple array of raw servo goal values.
  //
  for(i=0, nNumPts=0; i<trajectoryPoint.getNumPoints(); ++i)
  {
    //
    // Joint point data: joint name, goal position, velocity, and acceleration.
    //
    trajectoryPoint[i].get(strName, fPosition, fVelocity, fAcceleration);

    //
    // Kinematic joint
    //
    if( (pJoint = getJoint(strName)) == NULL )
    {
      LOGERROR("Joint %s: Not found in arm kinematic chain.", strName.c_str());
      return -PT_ECODE_BAD_VAL;
    }

    // get master servo
    PT_TRY_GET_SERVO(pJoint->m_nMasterServoId, pServo);

    //
    // Check for goal position out of range condition.
    //
    if( (pJoint->m_eJointType != PanTiltJointTypeContinuous) &&
        ((fPosition < pJoint->m_fMinSoftLimitRads) ||
         (fPosition > pJoint->m_fMaxSoftLimitRads)) )
    {
      LOGDIAG3("Joint %s (servo=%d): Goal position %.02lf: Out of range.",
          pJoint->m_strName.c_str(), pJoint->m_nMasterServoId,
          radToDeg(fPosition));

      // cap
      if( fPosition < pJoint->m_fMinSoftLimitRads )
      {
        fPosition = pJoint->m_fMinSoftLimitRads;
      }
      else
      {
        fPosition = pJoint->m_fMaxSoftLimitRads;
      }
    }
    
    // convert to raw values
    nOdPos    = jointPositionToOdPos(*pJoint, fPosition, units_radians);
    nRawSpeed = jointVelocityToRawSpeed(*pJoint, nOdPos, fVelocity,
                                                          units_percent);

    // test if effectively already at the goal position
    if( iabs(nOdPos - pServo->GetOdometer()) < TuneDeltaPos )
    {
      nRawSpeed = 0;
    }

    //
    // Zero speed. Just make sure the servo is stopped.
    //
    // Note:  Cannot send zero speed to servos in servo mode. It translates in
    //        the fw to the maximum uncontrolled speed. Bad.
    //
    if( nRawSpeed == 0 )
    {
      pServo->Stop();
      continue;
    }

    //
    // Add move tuple to array.
    //
    tupSpeedPos[nNumPts].m_nServoId = pJoint->m_nMasterServoId;
    tupSpeedPos[nNumPts].m_nPos     = nOdPos;
    tupSpeedPos[nNumPts].m_nSpeed   = nRawSpeed;

  }

  //
  // Now move.
  //
  if( nNumPts > 0 )
  {
    rc = m_pDynaChain->SyncMoveAtSpeedTo(tupSpeedPos, (uint_t)nNumPts);
    if( rc < 0 )
    {
      LOGERROR("Arm trajectory move: %s.\n", DynaStrError(rc));
      return -PT_ECODE_DYNA;
    }
    else
    {
      m_lastTraj = trajectoryPoint;
    }
  }
  else
  {
    rc = PT_OK;
  }

  return rc;
}

int PanTiltRobot::getRobotStatus(PanTiltRobotStatus &robotStatus)
{
  static int  TuneStoppedSpeed = 9;   ///< stopped speed

  int                 iter;       // servo iterator
  int                 nServoId;   // servo id
  DynaServo          *pServo;     // servo
  bool                bIsMoving;  // robot is [not] moving
  PanTiltServoHealth  health;     // servo health

  robotStatus.clear();

  robotStatus.m_eRobotMode = m_eRobotMode;
  robotStatus.m_eIsEStopped = m_bIsEStopped?
                                      PanTiltTriStateTrue: PanTiltTriStateFalse;
  robotStatus.m_eIsCalibrated = m_eOpState == PanTiltOpStateCalibrated?
                                      PanTiltTriStateTrue: PanTiltTriStateFalse;
  robotStatus.m_eAreDrivesPowered = m_bAreServosPowered? 
                                    PanTiltTriStateTrue: PanTiltTriStateFalse;

  robotStatus.m_eIsInError = PanTiltTriStateFalse;
  robotStatus.m_nErrorCode = PT_OK;
  bIsMoving                = false;

  // reset alarm state
  m_bAlarmState = m_bIsEStopped? true: false;

  //
  // Get servo states
  //
  for(nServoId = m_pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId = m_pDynaChain->IterNext(&iter))
  {
    pServo = m_pDynaChain->GetServo(nServoId);

    health.m_nServoId     = nServoId;
    health.m_fTemperature = pServo->CvtRawTempToC(pServo->GetCurTemp());
    health.m_fVoltage     = pServo->CvtRawVoltToVolts(pServo->GetCurVolt());
    health.m_uAlarms      = pServo->GetAlarms();

    if( health.m_uAlarms != DYNA_ALARM_NONE )
    {
      robotStatus.m_eIsInError = PanTiltTriStateTrue;
      robotStatus.m_nErrorCode = -PT_ECODE_ALARMED;
      m_bAlarmState            = true;
    }

    if( iabs(pServo->GetCurSpeed()) > TuneStoppedSpeed )
    {
      bIsMoving = true;
    }

    robotStatus.m_vecServoHealth.push_back(health);
  }

  // can only move if calibrated and no alarms
  if( (m_eOpState == PanTiltOpStateCalibrated) && !m_bAlarmState )
  {
    robotStatus.m_eIsMotionPossible = PanTiltTriStateTrue;
  }
  else
  {
    robotStatus.m_eIsMotionPossible = PanTiltTriStateFalse;
  }

  // estop overrides other alarms
  if( m_bIsEStopped )
  {
    robotStatus.m_eIsInError = PanTiltTriStateTrue;
    robotStatus.m_nErrorCode = -PT_ECODE_ESTOP;
  }

  robotStatus.m_eIsInMotion =
    bIsMoving? PanTiltTriStateTrue: PanTiltTriStateFalse;

  return PT_OK;
}

int PanTiltRobot::getJointState(PanTiltJointStatePoint &jointStatePoint)
{
  MapRobotJoints::iterator  iter;             // kinematic chain iterator
  int                       nMasterServoId;   // master servo id
  PanTiltRobotJoint        *pJoint;           // robotic joint
  DynaServo                *pServo;           // servo
  PanTiltJointState         jointState;       // working joint state
  byte_t                    uMask;            // working bit mask
  int                       i;                // working index

  PT_TRY_CONN();

  jointStatePoint.clear();

  jointStatePoint.setKinematicChainName("pan_tilt");

  //
  // Build joint state point.
  //
  for(iter = m_kin.begin(); iter != m_kin.end(); ++iter)
  {
    nMasterServoId  = iter->first;
    pJoint          = &(iter->second);

    PT_TRY_GET_SERVO(nMasterServoId, pServo);

    // identifiers
    jointState.m_strName        = pJoint->m_strName;
    jointState.m_eOpState       = pJoint->m_eOpState;
    jointState.m_nMasterServoId = nMasterServoId;
    jointState.m_nSlaveServoId  = pJoint->m_nSlaveServoId;

    // positions
    jointState.m_nOdPos    = pServo->GetOdometer();
    jointState.m_nEncPos   = pServo->OdometerToEncoder(jointState.m_nOdPos);
    jointState.m_fPosition = getCurJointPosition(*pJoint);

    // speeds and velocities
    jointState.m_nSpeed    = pServo->GetCurSpeed();
    jointState.m_fVelocity = getCurJointVelocity(*pJoint);

    // loads and effort
    jointState.m_fEffort = (double)pServo->GetCurLoad();

    // add state
    jointStatePoint.append(jointState);
  }

  return PT_OK;
}

int PanTiltRobot::getTrajectoryState(
                            PanTiltJointTrajectoryFeedback &trajectoryFeedback)
{
  PanTiltJointStatePoint  jointStatePoint;
  int                     iActual;
  int                     i;
  int                     rc;

  trajectoryFeedback.clear();

  // last issued trajectory is the desired trajectory
  trajectoryFeedback[PanTiltJointTrajectoryFeedback::TRAJ_DESIRED] = m_lastTraj;

  // retrieve joint state
  if( (rc = getJointState(jointStatePoint)) < 0 )
  {
    LOGERROR("Cannot retrieve joint state data.");
    return rc;
  }

  // copy joint state to actual trajectory
  iActual = PanTiltJointTrajectoryFeedback::TRAJ_ACTUAL;
 
  for(i=0; i<jointStatePoint.getNumPoints(); ++i)
  {
    trajectoryFeedback[iActual].append(jointStatePoint[i].m_strName,
                                       jointStatePoint[i].m_fPosition,
                                       jointStatePoint[i].m_fVelocity);
  }

  return PT_OK;
}

bool PanTiltRobot::isInMotion()
{
  static int      TuneStoppedSpeed = 9;   ///< stopped speed

  int             iter;       // servo iterator
  int             nServoId;   // servo id
  DynaServo      *pServo;     // servo

  for(nServoId = m_pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId = m_pDynaChain->IterNext(&iter))
  {
    pServo = m_pDynaChain->GetServo(nServoId);

    if( iabs(pServo->GetCurSpeed()) > TuneStoppedSpeed )
    {
      return true;
    }
  }

  return false;
}

int PanTiltRobot::jointPositionToOdPos(PanTiltRobotJoint &joint,
                                       double         fPosition,
                                       units_t        units)
{
  fPosition *= joint.m_fGearRatio;  // convert to servo position

  switch(units)
  {
    // radians
    case units_radians:
      return (int)(fPosition * joint.m_fTicksPerServoRad);

    // degrees
    case units_degrees:
      return (int)(degToRad(fPosition) * joint.m_fTicksPerServoRad);

    // raw servo units
    case units_raw:
    default:
      return (int)fPosition;
  }
}

int PanTiltRobot::jointVelocityToRawSpeed(PanTiltRobotJoint &joint,
                                          int            nOdGoalPos,
                                          double         fVelocity,
                                          units_t        units)
{
  int         nDir;
  DynaServo  *pServo;

  fVelocity *= joint.m_fGearRatio;    // convert to servo velocity

  if( fVelocity < 0.0 )
  {
    fVelocity = 0.0;
  }

  pServo = getMasterServo(joint);

  nDir = pServo->CalcSpeedDir(nOdGoalPos);

  switch(units)
  {
    // [0.0, 1.0]  of max servo speed
    case units_norm:
      if( fVelocity > 1.0 )
      {
        fVelocity = 1.0;
      }
      return nDir * (int)(fVelocity * (double)DYNA_SPEED_MAX_RAW);

    // % of max servo speed
    case units_percent:
      if( fVelocity > 100.0 )
      {
        fVelocity = 100.0;
      }
      return nDir * (int)(fVelocity/100.0 * (double)DYNA_SPEED_MAX_RAW);

    // %% of max servo speed
    case units_permil:
      if( fVelocity > 1000.0 )
      {
        fVelocity = 1000.0;
      }
      return nDir * (int)(fVelocity/1000.0 * (double)DYNA_SPEED_MAX_RAW);

    // radians per second
    case units_rad_per_s:
      if( fVelocity > joint.m_fMaxServoRadsPerSec )
      {
        fVelocity = joint.m_fMaxServoRadsPerSec;
      }
      return nDir * (int)(fVelocity / joint.m_fMaxServoRadsPerSec *
                          (double)DYNA_SPEED_MAX_RAW);

    // raw servo units
    case units_raw:
    default:
      if( fVelocity > (double)DYNA_SPEED_MAX_RAW )
      {
        fVelocity = (double)DYNA_SPEED_MAX_RAW;
      }
      return nDir * (int)fVelocity;
  }
}

double PanTiltRobot::getCurJointPosition(PanTiltRobotJoint &joint,
                                         units_t            units)
{
  double          fPosition;

  fPosition =  odPosToJointPosition(joint,
                                    getMasterServo(joint)->GetOdometer(),
                                    units);

  return fPosition;
}

double PanTiltRobot::odPosToJointPosition(PanTiltRobotJoint &joint,
                                          int            nOdPos,
                                          units_t        units)
{
  switch(units)
  {
    // raw servo units
    case units_raw:
      return (double)nOdPos;

    // degrees
    case units_degrees:
      return radToDeg((double)nOdPos / joint.m_fTicksPerJointRad);

    // radians
    case units_radians:
    default:
      return (double)nOdPos / joint.m_fTicksPerJointRad;
  }
}

double PanTiltRobot::getCurJointVelocity(PanTiltRobotJoint &joint,
                                         units_t            units)
{
  double          fVelocity;

  fVelocity =  rawSpeedToJointVelocity(joint,
                                       getMasterServo(joint)->GetCurSpeed(),
                                       units);

  return fVelocity;
}

double PanTiltRobot::rawSpeedToJointVelocity(PanTiltRobotJoint &joint,
                                             int            nSpeed,
                                             units_t        units)
{
  double fSpeed;
  double fGearRatio;

  fSpeed      = (double)iabs(nSpeed);
  fGearRatio  = joint.m_fGearRatio;

  switch(units)
  {
    // [0.0, 1.0]  of max servo speed
    case units_norm:
      return (fSpeed / (double)DYNA_SPEED_MAX_RAW) / fGearRatio;

    // % of max servo speed
    case units_percent:
      return (fSpeed / (double)DYNA_SPEED_MAX_RAW * 100.0) / fGearRatio;

    // %% of max servo speed
    case units_permil:
      return (fSpeed / (double)DYNA_SPEED_MAX_RAW * 1000.0) / fGearRatio;

    // radians per second
    case units_rad_per_s:
      return (fSpeed / (double)DYNA_SPEED_MAX_RAW) * 
              joint.m_fMaxServoRadsPerSec / fGearRatio;

    // raw servo units
    case units_raw:
    default:
      return fSpeed;
  }
}

int PanTiltRobot::scan()
{
  int   rc;

  if( (rc = scanDynaBus(3)) < 0 )
  {
    LOGERROR("Pan-Tilt dynamixel bus scan failed.");
    return rc;
  }

  m_bAlarmState = false;

  return PT_OK;
}

int PanTiltRobot::scanDynaBus(int nMaxTries)
{
  static uint_t usecTry = 500000;   // retry period
  static int    nMinServosReq = 2;  // absolute minimum number servos required 

  int   nNumServosExpected;   // number of expected and required servos
  int   nNumServosScanned;    // number of scanned servos
  int   nNumServosMatched;    // number of scanned and matched servos
  int   nTries;               // scan attempts
  int   nServoId;             // servo id
  int   iter;                 // servo iterator
  int   rc;                   // return code


  //
  // Scan hardware.
  //
  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    LOGDIAG3("Scanning Pan-Tilt hardware - attempt %d of %d.",
        nTries+1, nMaxTries);

    // scan for servos
    nNumServosScanned = m_pDynaChain->AddNewServosByScan();

    //
    // Scanned at least the required number.
    //
    if( nNumServosScanned >= nMinServosReq )
    {
      // set pan-tilt product description, given the servos in chain
      m_descPanTilt.setDesc(m_pDynaChain);

      // need a robot description before preceeding
      if( !m_descPanTilt.isDescribed() )
      {
        LOGERROR("Undefined Pan-Tilt description - "
                "don't know how to initialized properly.");
      }

      else
      {
        nNumServosExpected = m_descPanTilt.getNumServos();

        // check scanned ids against required
        for(nServoId = m_pDynaChain->IterStart(&iter), nNumServosMatched=0;
            nServoId != DYNA_ID_NONE;
            nServoId = m_pDynaChain->IterNext(&iter))
        {
          // matched
          if( m_descPanTilt.hasServo(nServoId) )
          {
            ++nNumServosMatched;
          }

          // unmathced
          else
          {
            LOGWARN("Servo %d unexpected - uncontrolled.", nServoId);
          }
        }

        // servos matched
        if( nNumServosMatched == nNumServosExpected )
        {
          rc = PT_OK;
          break;
        }

        // some required servos are missing
        else
        {
          LOGERROR("Match %d scanned servo ids, expected %d required matches.",
            nNumServosMatched, nNumServosExpected);
          rc = -PT_ECODE_FORMAT;
        }
      }
    }

    //
    // Scan failed to detect the minimum number of servos.
    //
    else
    {
      LOGERROR("Scanned %d servos, expected %d.",
          nNumServosScanned, nNumServosExpected);
      rc = -PT_ECODE_NO_RSRC;
    }

    usleep(usecTry);
  }

  if( rc == PT_OK )
  {
    LOGDIAG3("Pan-Tilt dynamixel bus hardware scanned successfully.");
  }

  return rc;
}


int PanTiltRobot::convertSpec()
{
  PanTiltSpecJoint_T *pSpecJoint;
  PanTiltSpecServo_T *pSpecServo;
  int                 nServoId;
  PanTiltRobotJoint  *pJoint;
  int                 i;
  int                 rc;

  m_kin.clear();
  m_imapKin.clear();

  //
  // Build up pan-tilt kinematic chain.
  //
  for(i = 0; i < m_descPanTilt.m_spec.m_nDoF; ++i)
  {
    // joint specification
    pSpecJoint = m_descPanTilt.m_spec.getJointSpecAt(i);

    // master servo id associated with joint
    nServoId  = pSpecJoint->m_nMasterServoId;
    pJoint    = &m_kin[nServoId];

    // servo specification
    if( (pSpecServo = m_descPanTilt.m_spec.getServoSpec(nServoId)) == NULL )
    {
      LOGERROR("Servo %d: Cannot find servo specification.", nServoId);
      return -PT_ECODE_NO_EXEC;
    }

    // add rebotic joint
    if( (rc = addRobotJoint(pSpecJoint, pSpecServo, m_kin, m_imapKin)) < 0 )
    {
      LOGERROR("Servo %d: Cannot add to kinematic chain.", nServoId);
      return rc;
    }
  }
 
  return PT_OK;
}

int PanTiltRobot::addRobotJoint(PanTiltSpecJoint_T *pSpecJoint,
                                PanTiltSpecServo_T *pSpecServo,
                                MapRobotJoints     &kin,
                                IMapRobotJoints    &imap)
{
  static double     TuneSpeedDerate = 0.25;   // derate scale of maximum speed

  int                 nMasterServoId;     // master servo id
  DynaServo          *pServo;             // servo
  uint_t              uTicks;             // ticks
  double              fAngleMin;          // min angle(deg) in servo mode
  double              fAngleMax;          // max angle(deg) in servo mode
  double              fMaxRpm;            // max unloaded rpm
  double              fTorqueMax;         // max torque(unitless)
  double              fDegrees;           // working degrees
  double              fRadians;           // working radians
  PanTiltRobotJoint   joint;              // robotic joint
  int                 i;                  // working index

  // master servo id associated with joint
  nMasterServoId = pSpecJoint->m_nMasterServoId;

  // dynamixel servo object in dynamixel chain
  if( (pServo = m_pDynaChain->GetServo(nMasterServoId)) == NULL )
  {
    LOGERROR("Servo %d: Cannot find servo in dynamixel chain.", nMasterServoId);
    return -PT_ECODE_NO_EXEC;
  }

  //
  // Dynamixel servo specification loaded during scan.
  //
  uTicks      = pServo->GetSpecification().m_uRawPosModulo;
  fAngleMin   = pServo->GetSpecification().m_fAngleMin;
  fAngleMax   = pServo->GetSpecification().m_fAngleMax;
  fMaxRpm     = pServo->GetSpecification().m_fMaxSpeed;
  fTorqueMax  = (double)pServo->GetSpecification().m_uRawTorqueMax;

  //
  // Servo rotation range.
  //
  fDegrees = pSpecServo->m_bIsContinuous? 360.0: fAngleMax - fAngleMin;
  fRadians = degToRad(fDegrees);

  //
  // Populate joint data.
  //
  // Some data, such as joint rotation limits may be refined during
  // calibration.
  //
  joint.m_strName             = pSpecJoint->m_strName;
  joint.m_nMasterServoId      = nMasterServoId;
  joint.m_nSlaveServoId       = pSpecJoint->m_nSlaveServoId;
  joint.m_bIsContinuous       = pSpecServo->m_bIsContinuous;
  joint.m_nMasterServoDir     = pSpecServo->m_nDir;
  joint.m_eJointType          = pSpecJoint->m_eJointType;
  joint.m_fGearRatio          = pSpecJoint->m_fGearRatio;
  joint.m_fTicksPerServoRad   = (double)uTicks / fRadians; 
  joint.m_fTicksPerJointRad   = joint.m_fTicksPerServoRad *
                                  pSpecJoint->m_fGearRatio;
  joint.m_fMaxServoRadsPerSec = (fMaxRpm * M_TAU / 60.0) * TuneSpeedDerate;
  joint.m_fMaxJointRadsPerSec = joint.m_fMaxServoRadsPerSec /
                                  joint.m_fGearRatio;
  joint.m_fMinPhyLimitRads    = degToRad(pSpecJoint->m_fMinPhyLimit);
  joint.m_fMaxPhyLimitRads    = degToRad(pSpecJoint->m_fMaxPhyLimit);
  joint.m_nMinPhyLimitOd      = (int)(joint.m_fTicksPerJointRad *
                                      joint.m_fMinPhyLimitRads);
  joint.m_nMaxPhyLimitOd      = (int)(joint.m_fTicksPerJointRad *
                                      joint.m_fMaxPhyLimitRads );
  joint.m_fMinSoftLimitRads   = joint.m_fMinPhyLimitRads;
  joint.m_fMaxSoftLimitRads   = joint.m_fMaxPhyLimitRads;
  joint.m_nMinSoftLimitOd     = joint.m_nMinPhyLimitOd;
  joint.m_nMaxSoftLimitOd     = joint.m_nMaxPhyLimitOd;
  joint.m_fCalibPosRads       = degToRad(pSpecJoint->m_fCalibPos);
  joint.m_eLimitTypes         = pSpecJoint->m_eLimitTypes;
  joint.m_uTorqueLimit        = (uint_t)(pSpecServo->m_fTorqueLimitPct / 100.0 *
                                     fTorqueMax);
  joint.m_eOpState            = PanTiltOpStateUncalibrated;
  joint.m_bStopAtOptLimits    = false;


  //
  // Sanity checks.
  //
  joint.m_fCalibPosRads = fcap(joint.m_fCalibPosRads,
                                joint.m_fMinPhyLimitRads,
                                joint.m_fMaxPhyLimitRads);

  //
  // Add to kinematic chain.
  //
  kin[nMasterServoId]   = joint;            // kinematic chain
  imap[joint.m_strName] = nMasterServoId;   // indirect map by joint name

  return PT_OK;
}

void PanTiltRobot::fauxcalibrate()
{
  freeze();
  configStateForAllServos(m_kin, false);
  resetOdometersForAllServos(m_kin);
  m_eOpState = PanTiltOpStateCalibrated;
}

int PanTiltRobot::configForCalibration(bool bForceRecalib)
{
  static double TuneOverTorqueTh  = 50.0;   // over torque threshold
  static double TuneClearTorqueTh = 45.0;   // clear over torque threshold

  int   rc;

  // stop any motions
  m_pDynaChain->Stop();

  // configure servo EEPROM for all servos
  if( (rc = configEEPROMForAllServos(m_kin)) < 0 )
  {
    LOGERROR("Failed to configure servo EEPROM for all servos.");
    return rc;
  }

  // configure servo RAM for all servos
  if( (rc = configRAMForAllServos(m_kin)) < 0 )
  {
    LOGERROR("Failed to configure servo RAM for all servos.");
    return rc;
  }

  // configure state for all servos
  if( (rc = configStateForAllServos(m_kin, bForceRecalib)) < 0 )
  {
    LOGERROR("Failed to configure state for all servos.");
    return rc;
  }

  // override torque values with gentler torque limits
  if( (rc = configSoftTorqueForAllServos(m_kin, TuneOverTorqueTh,
                                                   TuneClearTorqueTh)) < 0 )
  {
    LOGERROR(
        "Failed to configure calibration soft torque limits for all servos.");
    return rc;
  }

  // reset odometers for all servos
  if( (rc = resetOdometersForAllServos(m_kin)) < 0 )
  {
    LOGERROR("Failed to reset odometers for all servos.");
    return rc;
  }

  return PT_OK;
}

int PanTiltRobot::configForOperation()
{
  int   rc;

  m_pDynaChain->Stop();

  if( (rc = configEEPROMForAllServos(m_kin)) < 0 )
  {
    LOGERROR("Failed to configure servo EEPROM for all servos.");
    return rc;
  }

  if( (rc = configRAMForAllServos(m_kin)) < 0 )
  {
    LOGERROR("Failed to configure servo RAM for all servos.");
    return rc;
  }

  if( (rc = configStateForAllServos(m_kin, false)) < 0 )
  {
    LOGERROR("Failed to configure state for all servos.");
    return rc;
  }

  return PT_OK;
}

int PanTiltRobot::configEEPROMForAllServos(MapRobotJoints &kin)
{
  MapRobotJoints::iterator  iter;
  
  int     nServoId;
  int     rc;

  for(iter = kin.begin(); iter != kin.end(); ++iter)
  {
    nServoId = iter->second.m_nMasterServoId;

    if( (rc = configEEPROMForServo(nServoId, iter->second)) < 0 )
    {
      LOGERROR("Servo %d: Failed to configure servo EEPROM memory map.",
        nServoId);
      return rc;
    }

    nServoId = iter->second.m_nSlaveServoId;

    if( nServoId != DYNA_ID_NONE )
    {
      if( (rc = configEEPROMForServo(nServoId, iter->second)) < 0 )
      {
        LOGERROR("Servo %d: Failed to configure servo EEPROM memory map.",
          nServoId);
        return rc;
      }
    }
  }

  return PT_OK;
}

int PanTiltRobot::configEEPROMForServo(int nServoId, PanTiltRobotJoint &joint)
{
  DynaServo  *pServo;
  uint_t      uPosMin;
  uint_t      uPosMax;
  uint_t      uTorqueMax;
  uint_t      uVal;
  uint_t      uMask;
  int         rc;

  // dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  uPosMin     = pServo->GetSpecification().m_uRawPosMin;
  uPosMax     = pServo->GetSpecification().m_uRawPosMax;
  uTorqueMax  = pServo->GetSpecification().m_uRawTorqueMax;

  // --
  // Ensure servo is in the correct servo mode.
  // --
  
  // get servo must be in continuous mode
  if(joint.m_bIsContinuous && (pServo->GetServoMode() != DYNA_MODE_CONTINUOUS)) 
  {
    if( (rc = pServo->CfgWriteServoModeContinuous()) < 0 )
    {
      LOGERROR("Servo %d: Cannot configure EEPROM for continuous mode.",
          nServoId);
      return -PT_ECODE_DYNA;
    }
  }

  // servo must be in servo mode
  else if( !joint.m_bIsContinuous && 
           (pServo->GetServoMode() == DYNA_MODE_CONTINUOUS) )
  {
    if( (rc = pServo->CfgWriteServoMode(uPosMin, uPosMax)) < 0 )
    {
      LOGERROR("Servo %d: Cannot configure EEPROM for servo mode.", nServoId);
      return -PT_ECODE_DYNA;
    }
  }

  // --
  // Ensure maximum torque limit reload value is at the maximum.
  // --
  
  // always set maximum torque limit
  if( (rc = pServo->CfgReadMaxTorqueLimit(&uVal)) < 0 )
  {
    LOGWARN("Servo %d: Cannot read EEPROM maximum torque limit.", nServoId);
  }
  else if( uVal < uTorqueMax )
  {
    if( (rc = pServo->CfgWriteMaxTorqueLimit(uTorqueMax)) < 0 )
    {
      LOGWARN("Servo %d: Cannot configure EEPROM maximum torque limit.",
          nServoId);
    }
  }

  // --
  // Ensure alarm shutdown mask as the correct values.
  // --

  // for servos with no torque limit, disable over-load alarms
  if( joint.m_uTorqueLimit == 0 )
  {
    uMask = DYNA_ALARM_VOLTAGE | DYNA_ALARM_ANGLE | DYNA_ALARM_TEMP;
  }
  else
  {
    uMask = DYNA_ALARM_VOLTAGE | DYNA_ALARM_ANGLE | DYNA_ALARM_TEMP |
            DYNA_ALARM_LOAD;
  }

  if( (rc = pServo->CfgReadAlarmShutdownMask(&uVal)) < 0 )
  {
    LOGWARN("Servo %d: Cannot read EEPROM alarm shutdown mask.", nServoId);
  }
  else if( uVal != uMask )
  {
    if( (rc = pServo->CfgWriteAlarmShutdownMask(uMask)) < 0 )
    {
      LOGWARN("Servo %d: Cannot configure EEPROM alarm shutdown mask.",
          nServoId);
    }
  }

  return PT_OK;
}

int PanTiltRobot::configRAMForAllServos(MapRobotJoints &kin)
{
  MapRobotJoints::iterator  iter;
  int                       nServoId;
  int                       rc;

  for(iter = kin.begin(); iter != kin.end(); ++iter)
  {
    nServoId = iter->second.m_nMasterServoId;

    if( (rc = configRAMForServo(nServoId, iter->second)) < 0 )
    {
      LOGERROR("Servo %d: Failed to configure servo RAM memory map.",
        nServoId);
      return rc;
    }

    // no slave configuration
  }

  return PT_OK;
}

int PanTiltRobot::configRAMForServo(int nServoId, PanTiltRobotJoint &joint)
{
  DynaServo  *pServo;
  uint_t      uTorqueMax;
  int         rc;

  // get dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  uTorqueMax  = pServo->GetSpecification().m_uRawTorqueMax;

  // --
  // Ensure running torque limit is at the maximum. 
  // --
 
  if( (rc = pServo->WriteMaxTorqueLimit(uTorqueMax)) < 0 )
  {
    LOGWARN("Servo %d: Cannot write RAM alarm shutdown mask.", nServoId);
  }

  // --
  // Enable servo drive.
  // --

  if( (rc = pServo->WriteTorqueEnable(true)) < 0 )
  {
    LOGERROR("Servo %d: Cannot enable servo drive.", nServoId);
    return -PT_ECODE_DYNA;
  }

  return PT_OK;
}

int PanTiltRobot::configStateForAllServos(MapRobotJoints &kin,
                                          bool            bForceRecalib)
{
  MapRobotJoints::iterator    iterKin;
  int                         nServoId;
  int                         rc;

  for(iterKin = kin.begin(); iterKin != kin.end(); ++iterKin)
  {
    nServoId = iterKin->second.m_nMasterServoId;

    rc = configStateForServo(nServoId, true, bForceRecalib, iterKin->second);

    if( rc < 0 )
    {
      LOGERROR("Servo %d: Failed to configure servo vState memory map.",
        nServoId);
      return rc;
    }

    nServoId = iterKin->second.m_nSlaveServoId;

    if( nServoId != DYNA_ID_NONE )
    {
      rc = configStateForServo(nServoId, false, bForceRecalib, iterKin->second);

      if( rc < 0 )
      {
        LOGERROR("Servo %d: Failed to configure servo vState memory map.",
          nServoId);
        return rc;
      }
    }
  }

  return PT_OK;
}

int PanTiltRobot::configStateForServo(int                nServoId,
                                      bool               bIsMaster,
                                      bool               bForceRecalib,
                                      PanTiltRobotJoint &joint)
{
  DynaServo  *pServo;
  uint_t      uOverTorqueTh;
  uint_t      uClearTorqueTh;
  int         rc;

  // get dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  uOverTorqueTh   = joint.m_uTorqueLimit;
  uClearTorqueTh  = (uint_t)(0.90 * (double)uOverTorqueTh);

  if( (uOverTorqueTh > 0) && (uClearTorqueTh > 0) )
  {
    pServo->SetSoftTorqueThresholds(uOverTorqueTh, uClearTorqueTh);
  }

  if( bForceRecalib || (joint.m_eOpState != PanTiltOpStateCalibrated) )
  {
    joint.m_eOpState = PanTiltOpStateUncalibrated;
  }
}

int PanTiltRobot::configSoftTorqueForAllServos(MapRobotJoints &kin,
                                               double         fOverTorqueTh,
                                               double         fClearTorqueTh)
{
  MapRobotJoints::iterator  iter;
  int                       nServoId;
  DynaServo                *pServo;
  double                    fTorqueMax;
  uint_t                    uOverTorqueTh;
  uint_t                    uClearTorqueTh;

  fOverTorqueTh   /= 100.0;
  fClearTorqueTh  /= 100.0;

  for(iter = kin.begin(); iter != kin.end(); ++iter)
  {
    nServoId = iter->second.m_nMasterServoId;

    // get dynamixel servo object in dynamixel chain
    PT_TRY_GET_SERVO(nServoId, pServo);

    fTorqueMax      = (double)pServo->GetSpecification().m_uRawTorqueMax;
    uOverTorqueTh   = (uint_t)(fTorqueMax * fOverTorqueTh);
    uClearTorqueTh  = (uint_t)(fTorqueMax * fClearTorqueTh);

    if( (uOverTorqueTh > 0) && (uClearTorqueTh > 0) )
    {
      pServo->SetSoftTorqueThresholds(uOverTorqueTh, uClearTorqueTh);
    }
  }

  return PT_OK;
}

int PanTiltRobot::resetOdometersForAllServos(MapRobotJoints &kin)
{
  MapRobotJoints::iterator  iter;

  int         nServoId;
  bool        bIsReverse;
  int         rc;

  for(iter = kin.begin(), rc = PT_OK;
      (iter != kin.end()) && (rc == PT_OK);
      ++iter)
  {
    nServoId   = iter->second.m_nMasterServoId;
    bIsReverse = iter->second.m_nMasterServoDir == DYNA_DIR_CCW? false: true;

    rc = resetOdometerForServo(nServoId, bIsReverse);
  }

  return rc;
}

int PanTiltRobot::resetOdometerForServo(int nServoId, bool bIsReverse)
{
  DynaServo  *pServo;
  uint_t      uEncZeroPt;
  int         rc;

  // get dynamixel servo object in dynamixel chain
  PT_TRY_GET_SERVO(nServoId, pServo);

  if( (rc = pServo->Read(DYNA_ADDR_CUR_POS_LSB, &uEncZeroPt)) != DYNA_OK )
  {
    LOGERROR("Servo %d: Cannot read current encoder position.", nServoId);
    return -PT_ECODE_DYNA;
  }

  pServo->ResetOdometer(uEncZeroPt, bIsReverse);

  return PT_OK;
}

PanTiltOpState PanTiltRobot::detRobotOpState()
{
  MapRobotJoints::iterator  iter;

  for(iter = m_kin.begin(); iter != m_kin.end(); ++iter)
  {
    if( iter->second.m_eOpState != PanTiltOpStateCalibrated )
    {
      return PanTiltOpStateUncalibrated;
    }
  }
  return PanTiltOpStateCalibrated;
}

int PanTiltRobot::createAsyncThread()
{
  int   rc;

  m_eAsyncTaskState = PanTiltAsyncTaskStateWorking;

  rc = pthread_create(&m_threadAsync, NULL, PanTiltRobot::asyncThread,
                      (void*)this);
 
  if( rc == 0 )
  {
    rc = PT_OK;
  }

  else
  {
    m_eAsyncTaskState = PanTiltAsyncTaskStateIdle;
    LOGSYSERROR("pthread_create()");
    m_rcAsyncTask   = -PT_ECODE_SYS;
    m_eAsyncTaskId  = AsyncTaskIdNone;
    m_pAsyncTaskArg = NULL;
    rc = m_rcAsyncTask;
  }

  return rc;
}


void PanTiltRobot::cancelAsyncTask()
{
  MapRobotJoints::iterator  iter;

  if( m_eAsyncTaskState != PanTiltAsyncTaskStateIdle )
  {
    // cancel thread
    pthread_cancel(m_threadAsync);
    pthread_join(m_threadAsync, NULL);

    // cleanup
    switch( m_eAsyncTaskId )
    {
      case AsyncTaskIdCalibrate:
        freeze();
        for(iter = m_kin.begin(); iter != m_kin.end(); ++iter)
        {
          if( iter->second.m_eOpState != PanTiltOpStateCalibrated )
          {
            iter->second.m_eOpState = PanTiltOpStateUncalibrated;
            m_eOpState              = PanTiltOpStateUncalibrated;
          }
        }
        break;
      default:
        break;
    }

    // clear state
    m_eAsyncTaskId    = AsyncTaskIdNone;
    m_pAsyncTaskArg   = NULL;
    m_rcAsyncTask     = -PT_ECODE_INTR;
    m_eAsyncTaskState = PanTiltAsyncTaskStateIdle;
    LOGDIAG3("Async task canceled.");
  }
}

void *PanTiltRobot::asyncThread(void *pArg)
{
  PanTiltRobot *pThis = (PanTiltRobot *)pArg;
  int       oldstate;
  int       rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
  //pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &oldstate);

  LOGDIAG3("Async robot task thread created.");

  //
  // Execute asychronous task.
  //
  // For now, only calibrate task is supported asynchronously.
  //
  switch( pThis->m_eAsyncTaskId )
  {
    // Calibrate the robot. 
    case AsyncTaskIdCalibrate:
      {
        bool bForceRecalib = (bool)pThis->m_pAsyncTaskArg;
        rc = pThis->calibrate(bForceRecalib);
      }
      break;

    // Unknown task id.
    default:
      LOGERROR("Unknown async task id = %d.", (int)pThis->m_eAsyncTaskId);
      rc = -PT_ECODE_BAD_VAL;
      break;
  }

  // freeze robot at current calibrated or aborted position.
  //pThis->freeze();  disable, so that goto zero pt in calibration finsishes

  pThis->m_eAsyncTaskId     = AsyncTaskIdNone;
  pThis->m_pAsyncTaskArg    = NULL;
  pThis->m_rcAsyncTask      = rc;
  pThis->m_eAsyncTaskState  = PanTiltAsyncTaskStateIdle;

  LOGDIAG3("Async robot task thread exited.");

  return NULL;
}
