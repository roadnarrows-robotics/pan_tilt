////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptDesc.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltDesc - Pan-Tilt robotic mechanism description class interface.
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

#ifndef _PT_DESC_H
#define _PT_DESC_H

#include <vector>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#
#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptSpec.h"
#include "pan_tilt/ptProd.h"


namespace pan_tilt
{
  //
  // Forward declarations
  //
  class PanTiltDesc;
  class PanTiltRobot;

  /*!
   * \brief Pan-Tilt robotic mechanism description class.
   */
  class PanTiltDesc
  {
  public: 

    /*!
     * \breif Default constructor.
     */
    PanTiltDesc();
  
    /*!
     * \breif Destructor.
     */
    ~PanTiltDesc()
    {
    }
  
    /*
     * Set pan-tilt's description from what is discovered in the scanned
     * Dynamixel chain.
     *
     * \param pDynaChain  Scanned Dynamixel chain.
     */
    void setDesc(DynaChain *pDynaChain);

    /*!
     * \brief Reset base description to the "unitialized" values.
     */
    void resetDesc();

    /*!
     * \brief Test if required base description is adequately described.
     *
     * \return Returns true or false.
     */
    bool isDescribed()
    {
      return (m_eProdId != PanTiltProdIdUnknown)? true: false;
    }

    /*!
     * \brief Get this base description's base product id.
     *
     * \return Returns product id. See \ref PanTiltProdId.
     */
    int getProdId()
    {
      return m_eProdId;
    }
  
    /*!
     * \brief Get this base description's name.
     *
     * \return Returns string.
     */
    std::string getProdName()
    {
      return m_strProdName;
    }
  
    /*!
     * \brief Get this base description's brief.
     *
     * \return Returns string.
     */
    std::string getProdBrief()
    {
      return m_strProdBrief;
    }
  
    /*!
     * \brief Get this base description's hardware version.
     *
     * \return Returns string.
     */
    std::string getProdHwVer()
    {
      return m_strProdHwVer;
    }
  
    /*!
     * \brief Get this base description's size.
     *
     * \return Returns product size. See \ref PanTiltProdSize.
     */
    int getProdSize()
    {
      return m_eProdSize;
    }
  
    /*!
     * \brief Get this base description's degrees of freedom.
     *
     * \return Returns DoF.
     */
    int getDoF()
    {
      return m_nDoF;
    }

    /*!
     * \brief Get the number of expected and required servos.
     *
     * \return Returns number of servos.
     */
    int getNumServos()
    {
      return m_spec.getNumServos();
    }

    /*!
     * \brief Test if servo id is in the list of servos.
     *
     * \param nServoId    Servo id.
     *
     * \return Returns true or false.
     */
    bool hasServo(int nServoId)
    {
      return m_spec.hasServo(nServoId);
    }

    /*!
     * \brief Get the pan-tilt product name string given the product id.
     *
     * \param eProdId   Supported product id. See \ref PanTiltProdId.
     *
     * \return Returns product name string.
     * An unidentified product id returns "".
     */
    static const char *getProdName(int eProdId);
  
    /*!
     * \brief Get the pan-tilt product one-line brief description string.
     *
     * \param eProdId   Supported product id. See \ref PanTiltProdId.
     *
     * \return Returns product description string. An unidentified product id
     * returns "".
     */
    static const char *getProdBrief(int eProdId);
  
    /*!
     * \brief Get the pan-tilt size given the product id.
     *
     * \param eProdId   Supported product id. See \ref PanTiltProdId.
     *
     * \return Returns product size value.
     * An unidentified product id returns PanTiltProdSizeUknown.
     */
    static int getProdSize(int eProdId);

    /*!
     * \brief Get the pan-tilt degrees of freedom given the product id.
     *
     * \param eProdId   Supported product id. See \ref PanTiltProdId.
     *
     * \return Returns the degrees of freedom.
     */
    static int getDoF(int eProdId);

    /*!
     * \brief Get the pan-tilt hardware version string.
     *
     * \param eProdId   Supported product id. See \ref PanTiltProdId.
     *
     * \return Returns string.
     */
    static std::string getHwVer(int eProdId);

    /*!
     * \brief Get the \h_hek full brief descirption.
     *
     * \return Returns string.
     */
    std::string getFullProdBrief()
    {
      return m_strFullBrief;
    }
    
  protected: 
    bool          m_bIsDescribed;   ///< pan-tilt is [not] fully described
    int           m_eProdId;        ///< base product id
    std::string   m_strProdName;    ///< product name
    std::string   m_strProdBrief;   ///< product brief
    std::string   m_strProdHwVer;   ///< product hardware version string
    uint_t        m_uProdHwVer;     ///< product hardware version number
    int           m_eProdSize;      ///< product size code
    int           m_nDoF;           ///< degrees of freedom
    PanTiltSpec   m_spec;           ///< fixed specification
    std::string   m_strFullBrief;   ///< product with accessories full brief

    /*!
     * \brief Mark pan-tilt hardware as fully described.
     *
     * \copydoc doc_return_std
     */
    int markAsDescribed();
    
    friend class PanTiltRobot;
  };

} // namespace pan_tilt

#endif // _PT_DESC_H
