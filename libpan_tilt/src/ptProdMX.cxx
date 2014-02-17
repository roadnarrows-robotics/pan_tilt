////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptProdMX.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt 2 DOF mechanism with Dynamixel MX-28 servos.
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

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptSpec.h"
#include "pan_tilt/ptProdMX.h"

using namespace std;
using namespace pan_tilt;


/*!
 * \brief Specification of links.
 *
 * \par Data:
 * link_name,\n
 * length(mm)
 *
 * TODO
 */
const PanTiltSpecLink_T pan_tilt::PanTiltProdMXSpecLinks[PanTiltProdMXNumLinks] =
{
  // fixed base to pan
  { "base_fixed",
    0.0
  },

  // pan to tilt
  { "pan",
    0.0
  },

  // tilt to tool zero point
  { "tool_zero",
    0.0
  }
};

/*!
 * \brief Specification of joints.
 *
 * \par Data:
 * joint_name,\n
 * master_servo_id, slave_servo_id, joint_type, gear_ratio,\n
 * min_phy_limit(deg), max_phy_limit(deg), limit_types,\n
 * calib_pos(deg)
 * parent_link_idx, child_link_index
 */
const PanTiltSpecJoint_T  pan_tilt::PanTiltProdMXSpecJoints[PanTiltProdMXDoF] =
{
  { "pan",
    PanTiltServoIdPan, DYNA_ID_NONE, PanTiltJointTypeRevolute, 1.0,
    -170.0, 170.0, PanTiltLimitTypePhys,
    0.0,
    0, 1
  },

  { "tilt",
    PanTiltServoIdTilt, DYNA_ID_NONE, PanTiltJointTypeRevolute, 1.0,
    -90.0, 90.0, PanTiltLimitTypePhys,
    0.0,
    1, 2
  },
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const PanTiltSpecServo_T pan_tilt::PanTiltProdMXSpecServos[PanTiltProdMXNumServos] =
{
  {PanTiltServoIdPan,   true, true, DYNA_DIR_CW,  60.0},
  {PanTiltServoIdTilt,  true, true, DYNA_DIR_CW,  60.0}
};
