////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptDesc.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltDesc - Pan-Tilt robotic mechanism description class
 * implementation.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2014  RoadNarrows
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

#include <stdio.h>
#include <math.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/AX.h"
#include "Dynamixel/MX.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"
#include "pan_tilt/ptSpec.h"
#include "pan_tilt/ptProd.h"
#include "pan_tilt/ptDesc.h"

using namespace std;
using namespace pan_tilt;


PanTiltDesc::PanTiltDesc()
{
  m_eProdId     = PanTiltProdIdUnknown;
  m_eProdSize   = PanTiltProdSizeUnknown; 
  m_uProdHwVer  = 0;
  m_nDoF        = 0;
}

void PanTiltDesc::setDesc(DynaChain *pDynaChain)
{
  int         nServoIdPan;
  int         nServoIdTilt;
  DynaServo  *pServo;

  if( (pServo = pDynaChain->GetServo(PanTiltServoIdPan)) != NULL )
  {
    nServoIdPan  = PanTiltServoIdPan;
    nServoIdTilt = PanTiltServoIdTilt;
  }
  else if( (pServo = pDynaChain->GetServo(PanTiltEquipServoIdPan)) != NULL )
  {
    nServoIdPan  = PanTiltEquipServoIdPan;
    nServoIdTilt = PanTiltEquipServoIdTilt;
  }
  else if( (pServo = pDynaChain->GetServo(PanTiltAuxServoIdPan)) != NULL )
  {
    nServoIdPan  = PanTiltAuxServoIdPan;
    nServoIdTilt = PanTiltAuxServoIdTilt;
  }
  else
  {
    LOGERROR("Pan servo %d|%d|%d not found.",
        PanTiltServoIdPan, PanTiltEquipServoIdPan, PanTiltAuxServoIdPan);
    return;
  }

  switch( pServo->GetConfiguration().m_uModelNum )
  {
    case DYNA_MODEL_NUM_AX12:
      m_eProdId = PanTiltProdAXId;
      break;
    case DYNA_MODEL_NUM_MX28:
      m_eProdId = PanTiltProdMXId;
      break;
    default:
      LOGERROR("Unknown Pan-Tilt product with servo module 0x%04x.",
              pServo->GetConfiguration().m_uModelNum);
      return;
  }

  m_strProdName   = getProdName(m_eProdId);
  m_strProdBrief  = getProdBrief(m_eProdId);
  m_strProdHwVer  = getHwVer(m_eProdId);
  m_uProdHwVer    = strToVersion(m_strProdHwVer);
  m_nDoF          = getDoF();
  m_eProdSize     = getProdSize();

  // set specification
  m_spec.set(m_eProdId, m_uProdHwVer, nServoIdPan, nServoIdTilt);

  markAsDescribed();
}

void PanTiltDesc::resetDesc()
{
  m_eProdId       = PanTiltProdIdUnknown;
  m_strProdName.clear();
  m_strProdBrief.clear();
  m_strProdHwVer.clear();
  m_nDoF          = 0;
  m_eProdSize     = PanTiltProdSizeUnknown;
  m_spec.clear();
  m_strFullBrief.clear();
  m_bIsDescribed = false;
}

const char *PanTiltDesc::getProdName(int eProdId)
{
  switch( eProdId )
  {
    case PanTiltProdAXId:
      return "PanTilt-AX";
    case PanTiltProdMXId:
      return "PanTilt-MX";
    default:
      return "";
  }
}

const char *PanTiltDesc::getProdBrief(int eProdId)
{
  switch( eProdId )
  {
    case PanTiltProdAXId:
      return "RoadNarrows Pan-Tilt 2DOF AX-12 robotic mechanism";
    case PanTiltProdMXId:
      return "RoadNarrows Pan-Tilt 2DOF MX-28 robotic mechanism";
    default:
      return "";
  }
}

int PanTiltDesc::getProdSize(int eProdId)
{
  switch( eProdId )
  {
    case PanTiltProdAXId:
    case PanTiltProdMXId:
      return PanTiltProdSizeStd;
    default:
      return PanTiltProdSizeUnknown;
  }
}

int PanTiltDesc::getDoF(int eProdId)
{
  switch( eProdId )
  {
    case PanTiltProdAXId:
    case PanTiltProdMXId:
      return 2;
    default:
      return 0;
  }
}

string PanTiltDesc::getHwVer(int eProdId)
{
  int   nVersion;
  char  buf[256];

  switch( eProdId )
  {
    case PanTiltProdAXId:
      nVersion = PanTiltProdAXVersion;
      break;
    case PanTiltProdMXId:
      nVersion = PanTiltProdMXVersion;
      break;
    default:
      nVersion = 0;
  }

  sprintf(buf, "%d.%d.%d",
        PT_VER_MAJOR(nVersion), PT_VER_MINOR(nVersion), PT_VER_REV(nVersion));

  return string(buf);
}

int PanTiltDesc::markAsDescribed()
{
  m_strFullBrief =  getProdBrief();

  m_bIsDescribed = true;

  return PT_OK;
}
