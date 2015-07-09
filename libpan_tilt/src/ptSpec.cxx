////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptSpec.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt product specification base implimentations.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
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

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptSpec.h"
#include "pan_tilt/ptProd.h"

using namespace std;
using namespace pan_tilt;


PanTiltSpec::PanTiltSpec()
{
  m_eProdFamily   = PanTiltProdFamilyUnknown;
  m_eProdId       = PanTiltProdIdUnknown;
  m_uHwVer        = 0;
  m_nNumLinks     = 0;
  m_nDoF          = 0;
  m_nNumOptLimits = 0;
  m_nNumServos    = 0;
}

PanTiltSpec::~PanTiltSpec()
{
}

int PanTiltSpec::set(int    eProdId,
                     uint_t uHwVer,
                     int    nServoIdPan,
                     int    nServoIdTilt)
{
  const PanTiltSpecLink_T  *pSpecLinks;
  const PanTiltSpecJoint_T *pSpecJoints;
  const PanTiltSpecServo_T *pSpecServos;
  PanTiltSpecJoint_T        specJoint;
  PanTiltSpecServo_T        specServo;
  int                       i;

  clear();

  switch( eProdId )
  {
    //
    // AX Pan-Tilt
    //
    case PanTiltProdAXId:
      m_eProdFamily   = PT_PROD_FAMILY;
      m_nNumLinks     = PanTiltProdAXNumLinks;
      m_nDoF          = PanTiltProdAXDoF;
      m_nNumOptLimits = PanTiltProdAXNumOptLimits;
      m_nNumServos    = PanTiltProdAXNumServos;
      pSpecLinks      = PanTiltProdAXSpecLinks;
      pSpecJoints     = PanTiltProdAXSpecJoints;
      pSpecServos     = PanTiltProdAXSpecServos;
      break;

    //
    // MX Pan-Tilt
    //
    case PanTiltProdMXId:
      m_eProdFamily   = PT_PROD_FAMILY;
      m_nNumLinks     = PanTiltProdMXNumLinks;
      m_nDoF          = PanTiltProdMXDoF;
      m_nNumOptLimits = PanTiltProdMXNumOptLimits;
      m_nNumServos    = PanTiltProdMXNumServos;
      pSpecLinks      = PanTiltProdMXSpecLinks;
      pSpecJoints     = PanTiltProdMXSpecJoints;
      pSpecServos     = PanTiltProdMXSpecServos;
      break;

    // error
    default:
      LOGERROR("0x%08x: Unknown or unsupported product id.", eProdId);
      return -PT_ECODE_BAD_VAL;
  }

  //
  // Load link specifications
  //
  if( pSpecLinks != NULL )
  {
    for(i=0; i<m_nNumLinks; ++i)
    {
      m_vecSpecLinks.push_back(pSpecLinks[i]);
    }
  }

  // 
  // Load joint specification making any necessary run-time modifications.
  //
  if( pSpecJoints != NULL )
  {
    for(i=0; i<m_nDoF; ++i)
    {
      specJoint = pSpecJoints[i];

      if( specJoint.m_strName == "pan" )
      {
        specJoint.m_nMasterServoId = nServoIdPan;
      }
      else if( specJoint.m_strName == "tilt" )
      {
        specJoint.m_nMasterServoId = nServoIdTilt;
      }

      m_vecSpecJoints.push_back(specJoint);
    }
  }

  // 
  // Load servo specification making any necessary run-time modifications.
  //
  if( pSpecServos != NULL )
  {
    for(i=0; i<m_nNumServos; ++i)
    {
      specServo = pSpecServos[i];

      if( specServo.m_nServoId == PanTiltServoIdPan )
      {
        specServo.m_nServoId = nServoIdPan;
      }
      else if( specServo.m_nServoId == PanTiltServoIdTilt )
      {
        specServo.m_nServoId = nServoIdTilt;
      }

      m_vecSpecServos.push_back(specServo);
    }
  }

  m_eProdId     = eProdId;
  m_uHwVer      = uHwVer;

  return PT_OK;
}

void PanTiltSpec::clear()
{
  m_eProdId = PanTiltProdIdUnknown;
  m_uHwVer  = 0;
  m_vecSpecLinks.clear();
  m_vecSpecJoints.clear();
  m_vecSpecServos.clear();
}

PanTiltSpecJoint_T *PanTiltSpec::getJointSpec(string &strName)
{
  vector<PanTiltSpecJoint_T>::iterator    iter;

  for(iter = m_vecSpecJoints.begin(); iter != m_vecSpecJoints.end(); ++iter)
  {
    if( iter->m_strName == strName )
    {
      return &(*iter);
    }
  }

  return NULL;
}

PanTiltSpecJoint_T *PanTiltSpec::getJointSpec(int nServoId)
{
  vector<PanTiltSpecJoint_T>::iterator    iter;

  for(iter = m_vecSpecJoints.begin(); iter != m_vecSpecJoints.end(); ++iter)
  {
    if( iter->m_nMasterServoId == nServoId )
    {
      return &(*iter);
    }
  }

  return NULL;
}

PanTiltSpecServo_T *PanTiltSpec::getServoSpec(int nServoId)
{
  vector<PanTiltSpecServo_T>::iterator    iter;

  for(iter = m_vecSpecServos.begin(); iter != m_vecSpecServos.end(); ++iter)
  {
    if( iter->m_nServoId == nServoId )
    {
      return &(*iter);
    }
  }

  return NULL;
}

bool PanTiltSpec::hasServo(int nServoId)
{
  vector<PanTiltSpecServo_T>::iterator iter;

  for(iter = m_vecSpecServos.begin(); iter != m_vecSpecServos.end(); ++iter)
  {
    if( iter->m_nServoId == nServoId )
    {
      return true;
    }
  }
  return false;
}
