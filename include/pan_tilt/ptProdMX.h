////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptProdMX.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt 2 DOF mechanism with MX-28 servos.
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

#ifndef _PT_PROD_MX_H
#define _PT_PROD_MX_H

#include "Dynamixel/Dynamixel.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptSpec.h"


/*!
 * \ingroup pt_spec
 * \defgroup pt_spec_ax Pan-Tilt MX 2 DOF Mechanism
 *
 * \{
 */

/*! product id */
#define PT_MX_PRODUCT_ID   2

/*! hardware version */
#define PT_MX_VERSION      PT_VERSION(1, 0, 0)

namespace pan_tilt
{
  const int PanTiltProdMXId           = PT_MX_PRODUCT_ID; ///< product id
  const int PanTiltProdMXVersion      = PT_MX_VERSION;    ///< hw version

  const int PanTiltProdMXNumLinks     = 3;  ///< number of fixed links
  const int PanTiltProdMXDoF          = 2;  ///< degrees of freedom 
  const int PanTiltProdMXNumOptLimits = 0;  ///< number of optical limits
  const int PanTiltProdMXNumServos    = 2;  ///< number of servos

  /*!
   * \brief Specification of links.
   */
  extern const PanTiltSpecLink_T PanTiltProdMXSpecLinks[];

  /*!
   * \brief Specification of joints.
   */
  extern const PanTiltSpecJoint_T  PanTiltProdMXSpecJoints[];

  /*!
   * \brief Specification of servos.
   */
  extern const PanTiltSpecServo_T PanTiltProdMXSpecServos[];


} // namespace pan_tilt

/*! \} */


#endif // _PT_PROD_MX_H
