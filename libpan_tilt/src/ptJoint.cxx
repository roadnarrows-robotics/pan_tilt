////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptJoint.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt robot Joint classes implementations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#include <stdio.h>
#include <unistd.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"
#include "pan_tilt/ptSpec.h"
#include "pan_tilt/ptJoint.h"

using namespace std;
using namespace pan_tilt;


// -----------------------------------------------------------------------------
// Class PanTiltRobotJoint
// -----------------------------------------------------------------------------

PanTiltRobotJoint::PanTiltRobotJoint()
{
  // (derived) specification
  m_nMasterServoId      = DYNA_ID_NONE;
  m_nSlaveServoId       = DYNA_ID_NONE;
  m_bIsContinuous       = false;
  m_nMasterServoDir     = DYNA_DIR_NONE;
  m_eJointType          = PanTiltJointTypeUnknown;
  m_fGearRatio          = 1.0;
  m_fTicksPerServoRad   = 0.0;
  m_fTicksPerJointRad   = 0.0;

  // discovered limits and positions
  m_fMaxServoRadsPerSec = 0.0;
  m_fMaxJointRadsPerSec = 0.0;

  m_nEncZeroPos         = 0;
  m_fMinPhyLimitRads    = 0.0;
  m_fMaxPhyLimitRads    = 0.0;
  m_nMinPhyLimitOd      = 0;
  m_nMaxPhyLimitOd      = 0;

  m_fMinSoftLimitRads   = 0.0;
  m_fMaxSoftLimitRads   = 0.0;
  m_nMinSoftLimitOd     = 0;
  m_nMaxSoftLimitOd     = 0;

  m_fCalibPosRads       = 0.0;

  m_eLimitTypes         = PanTiltLimitTypeUnknown;
  m_uTorqueLimit        = 0;

  // state
  m_eOpState            = PanTiltOpStateUncalibrated;
  m_bStopAtOptLimits    = true;
}

PanTiltRobotJoint::PanTiltRobotJoint(const PanTiltRobotJoint &src)
{
  // (derived) specification
  m_strName             = src.m_strName;
  m_nMasterServoId      = src.m_nMasterServoId;
  m_nSlaveServoId       = src.m_nSlaveServoId;
  m_bIsContinuous       = src.m_bIsContinuous;
  m_nMasterServoDir     = src.m_nMasterServoDir;
  m_eJointType          = src.m_eJointType;
  m_fGearRatio          = src.m_fGearRatio;
  m_fTicksPerServoRad   = src.m_fTicksPerServoRad;
  m_fTicksPerJointRad   = src.m_fTicksPerJointRad;

  // discovered limits and positions
  m_fMaxServoRadsPerSec = src.m_fMaxServoRadsPerSec;
  m_fMaxJointRadsPerSec = src.m_fMaxJointRadsPerSec;

  m_nEncZeroPos         = src.m_nEncZeroPos;
  m_fMinPhyLimitRads    = src.m_fMinPhyLimitRads;
  m_fMaxPhyLimitRads    = src.m_fMaxPhyLimitRads;
  m_nMinPhyLimitOd      = src.m_nMinPhyLimitOd;
  m_nMaxPhyLimitOd      = src.m_nMaxPhyLimitOd;

  m_fMinSoftLimitRads   = src.m_fMinSoftLimitRads;
  m_fMaxSoftLimitRads   = src.m_fMaxSoftLimitRads;
  m_nMinSoftLimitOd     = src.m_nMinSoftLimitOd;
  m_nMaxSoftLimitOd     = src.m_nMaxSoftLimitOd;

  m_fCalibPosRads       = src.m_fCalibPosRads;

  m_eLimitTypes         = src.m_eLimitTypes;
  m_uTorqueLimit        = src.m_uTorqueLimit;

  // state
  m_eOpState            = src.m_eOpState;
  m_bStopAtOptLimits    = src.m_bStopAtOptLimits;
}

PanTiltRobotJoint::~PanTiltRobotJoint()
{
}

PanTiltRobotJoint PanTiltRobotJoint::operator=(const PanTiltRobotJoint &rhs)
{
  // (derived) specification
  m_strName             = rhs.m_strName;
  m_nMasterServoId      = rhs.m_nMasterServoId;
  m_nSlaveServoId       = rhs.m_nSlaveServoId;
  m_bIsContinuous       = rhs.m_bIsContinuous;
  m_nMasterServoDir     = rhs.m_nMasterServoDir;
  m_eJointType          = rhs.m_eJointType;
  m_fGearRatio          = rhs.m_fGearRatio;
  m_fTicksPerServoRad   = rhs.m_fTicksPerServoRad;
  m_fTicksPerJointRad   = rhs.m_fTicksPerJointRad;

  // discovered limits and positions
  m_fMaxServoRadsPerSec = rhs.m_fMaxServoRadsPerSec;
  m_fMaxJointRadsPerSec = rhs.m_fMaxJointRadsPerSec;

  m_nEncZeroPos         = rhs.m_nEncZeroPos;
  m_fMinPhyLimitRads    = rhs.m_fMinPhyLimitRads;
  m_fMaxPhyLimitRads    = rhs.m_fMaxPhyLimitRads;
  m_nMinPhyLimitOd      = rhs.m_nMinPhyLimitOd;
  m_nMaxPhyLimitOd      = rhs.m_nMaxPhyLimitOd;

  m_fMinSoftLimitRads   = rhs.m_fMinSoftLimitRads;
  m_fMaxSoftLimitRads   = rhs.m_fMaxSoftLimitRads;
  m_nMinSoftLimitOd     = rhs.m_nMinSoftLimitOd;
  m_nMaxSoftLimitOd     = rhs.m_nMaxSoftLimitOd;

  m_fCalibPosRads       = rhs.m_fCalibPosRads;

  m_eLimitTypes         = rhs.m_eLimitTypes;
  m_uTorqueLimit        = rhs.m_uTorqueLimit;

  // state
  m_eOpState            = rhs.m_eOpState;
  m_bStopAtOptLimits    = rhs.m_bStopAtOptLimits;

  return *this;
}


// -----------------------------------------------------------------------------
// Class PanTiltJointState
// -----------------------------------------------------------------------------

PanTiltJointState::PanTiltJointState()
{
  m_eOpState        = PanTiltOpStateUncalibrated;
  m_nMasterServoId  = DYNA_ID_NONE;
  m_nSlaveServoId   = DYNA_ID_NONE;
  m_fPosition       = 0.0;
  m_fVelocity       = 0.0;
  m_fEffort         = 0.0;
  m_nOdPos          = 0;
  m_nEncPos         = 0;
  m_nSpeed          = 0;
}

PanTiltJointState::PanTiltJointState(const PanTiltJointState &src)
{
  m_strName         = src.m_strName;
  m_eOpState        = src.m_eOpState;
  m_nMasterServoId  = src.m_nMasterServoId;
  m_nSlaveServoId   = src.m_nSlaveServoId;
  m_fPosition       = src.m_fPosition;
  m_fVelocity       = src.m_fVelocity;
  m_fEffort         = src.m_fEffort;
  m_nOdPos          = src.m_nOdPos;
  m_nEncPos         = src.m_nEncPos;
  m_nSpeed          = src.m_nSpeed;
}

PanTiltJointState PanTiltJointState::operator=(const PanTiltJointState &rhs)
{
  m_strName         = rhs.m_strName;
  m_eOpState        = rhs.m_eOpState;
  m_nMasterServoId  = rhs.m_nMasterServoId;
  m_nSlaveServoId   = rhs.m_nSlaveServoId;
  m_fPosition       = rhs.m_fPosition;
  m_fVelocity       = rhs.m_fVelocity;
  m_fEffort         = rhs.m_fEffort;
  m_nOdPos          = rhs.m_nOdPos;
  m_nEncPos         = rhs.m_nEncPos;
  m_nSpeed          = rhs.m_nSpeed;

  return *this;
}


// -----------------------------------------------------------------------------
// Class PanTiltJointStatePoint
// -----------------------------------------------------------------------------

static PanTiltJointState  nojointstate;

bool PanTiltJointStatePoint::hasJoint(const string &strJointName)
{
  vector<PanTiltJointState>::iterator iter;

  for(iter = m_jointState.begin(); iter != m_jointState.end(); ++iter)
  {
    if( iter->m_strName == strJointName )
    {
      return true;
    }
  }
  return false;
}

PanTiltJointState &PanTiltJointStatePoint::operator[](
                                                    const string &strJointName)
{
  vector<PanTiltJointState>::iterator iter;

  for(iter = m_jointState.begin(); iter != m_jointState.end(); ++iter)
  {
    if( iter->m_strName == strJointName )
    {
      return *iter;
    }
  }
  return nojointstate;
}
