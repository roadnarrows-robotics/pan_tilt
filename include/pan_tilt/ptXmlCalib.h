////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptXmlCalib.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltXmlCalib - The Pan-Tilt calibration data class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows LLC
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

#ifndef _PT_XML_CALIB_H
#define _PT_XML_CALIB_H

#include <string>

#include "tinyxml.h"

#include "rnr/rnrconfig.h"
#include "rnr/appkit/Xml.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"


namespace pan_tilt
{
  //----------------------------------------------------------------------------
  // PanTiltCalibParams Class
  //----------------------------------------------------------------------------

  /*!
   * \brief PanTiltCalibParams  Pan-Tilt XML parameters data class.
   */
  class PanTiltCalibParams
  {
  public:
    static const int UNSPEC = -999999;    ///< unspecified parameter value

    bool  m_bIsSpecified; ///< calibration parameters [not] fully specified
    int   m_nPanEncZero;  ///< pan joint zero point encoder value
    int   m_nPanOdMin;    ///< pan joint minimum odometer value at zero pos
    int   m_nPanOdMax;    ///< pan joint maximum odometer value at zero pos
    int   m_nTiltEncZero; ///< tilt joint zero point encoder value
    int   m_nTiltOdMin;   ///< tilt joint minimum odometer value at zero pos
    int   m_nTiltOdMax;   ///< tilt joint maximum odometer value at zero pos

    /*!
     * \brief Default constructor.
     */
    PanTiltCalibParams()
    {
      m_bIsSpecified  = false;
      m_nPanOdMin     = UNSPEC;
      m_nPanEncZero   = UNSPEC;
      m_nPanOdMax     = UNSPEC;
      m_nTiltOdMin    = UNSPEC;
      m_nTiltEncZero  = UNSPEC;
      m_nTiltOdMax    = UNSPEC;
    }

    /*!
     * \brief Copy constructor.
     */
    PanTiltCalibParams(const PanTiltCalibParams &src)
    {
      m_bIsSpecified  = src.m_bIsSpecified;
      m_nPanEncZero   = src.m_nPanEncZero;
      m_nPanOdMin     = src.m_nPanOdMin;
      m_nPanOdMax     = src.m_nPanOdMax;
      m_nTiltEncZero  = src.m_nTiltEncZero;
      m_nTiltOdMin    = src.m_nTiltOdMin;
      m_nTiltOdMax    = src.m_nTiltOdMax;
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    PanTiltCalibParams operator=(const PanTiltCalibParams &rhs)
    {
      m_bIsSpecified  = rhs.m_bIsSpecified;
      m_nPanEncZero   = rhs.m_nPanEncZero;
      m_nPanOdMin     = rhs.m_nPanOdMin;
      m_nPanOdMax     = rhs.m_nPanOdMax;
      m_nTiltEncZero  = rhs.m_nTiltEncZero;
      m_nTiltOdMin    = rhs.m_nTiltOdMin;
      m_nTiltOdMax    = rhs.m_nTiltOdMax;
    }
  };

  //----------------------------------------------------------------------------
  // PanTiltXmlCalib Class
  //----------------------------------------------------------------------------

  /*!
   * \brief PanTiltXmlCalib Pan-Tilt XML configuration class.
   */
  class PanTiltXmlCalib : public rnr::Xml
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PanTiltXmlCalib() : 
      Xml("pantilt", PanTiltXsiUrl, PanTiltXslUrl),
      m_strMajElemCalib("calibration"),
      m_strSecElemPan("pan"),
      m_strSecElemTilt("tilt"),
      m_strElemMin("min"),
      m_strElemZero("zero"),
      m_strElemMax("max")
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~PanTiltXmlCalib()
    {
    }

    /*!
     * \brief Load XML file into DOM and set the Pan-Tilt calibration
     * parameters.
     *
     * \param [out] params    Pan-Tilt calibration parameters.
     * \param strSearchPath   Search path of directory paths.
     * \param strXmlFileName  XML file name.
     * \param bAllInstances   Do [not] load and set all instances of XML files
     *                        found.
     *
     * \copydoc doc_return_std
     */
    virtual int load(PanTiltCalibParams &params,
                     const std::string  &strSearchPath=PanTiltUserCfgPath,
                     const std::string  &strXmlFileName=PanTiltCalibXml,
                     bool               bAllInstances=false);

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // I/O Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Load XML file into DOM.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(const std::string &strXmlFileName=PanTiltCalibXml);

    /*!
     * \brief Load XML file into DOM and set the Pan-Tilt calibration
     * parameters.
     *
     * \param [out] params    Pan-Tilt calibration parameters.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(PanTiltCalibParams           &params,
                         const std::string &strXmlFileName=PanTiltCalibXml);

    /*!
     * \brief Save DOM to XML file.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const std::string &strXmlFileName=PanTiltCalibXml);

    /*!
     * \brief Set DOM from Pan-Tilt calibration parameters and save XML file.
     *
     * \param [in] params     Pan-Tilt calibration parameters.
     * \param strDirName      XML output directory name.
     * \param strXmlFileName  XML file name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const PanTiltCalibParams &params,
                         const std::string &strDirName=PanTiltUserCfgDir,
                         const std::string &strXmlFileName=PanTiltCalibXml);

    /*!
     * \brief Set the Pan-Tilt calibration parameters from the DOM.
     *
     * \param [out] params    Pan-Tilt calibration parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int setPanTiltParamsFromDOM(PanTiltCalibParams &params);

    /*!
     * \brief Set the DOM from the Pan-Tilt calibration parameters.
     *
     * \param [in] params   Pan-Tilt calibration parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int setDOMFromPanTiltParams(const PanTiltCalibParams &params);

  protected:
    std::string m_strMajElemCalib;  ///< calibration major element name
    std::string m_strSecElemPan;    ///< pan joint section element name
    std::string m_strSecElemTilt;   ///< tilt joint section element name
    std::string m_strElemMin;       ///< minimum element name
    std::string m_strElemZero;      ///< zero point element name
    std::string m_strElemMax;       ///< maximum element name

    /*!
     * \brief Set the pan joint parameters from the DOM.
     *
     * \param pElemMaj      Pointer to major DOM base element.
     * \param [out] params  Pan-Tilt calibration parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int setPanParamsFromDOM(TiXmlElement       *pElemMaj,
                                    PanTiltCalibParams &params);

    /*!
     * \brief Set the tilt joint parameters from the DOM.
     *
     * \param pElemMaj      Pointer to major DOM base element.
     * \param [out] params  Pan-Tilt calibration parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int setTiltParamsFromDOM(TiXmlElement       *pElemMaj,
                                     PanTiltCalibParams &params);

    /*!
     * \brief Warn on unknown element.
     *
     * \param strElem   Element name.
     */
    void warnUnknownElem(const std::string &strElem)
    {
      setErrorMsg("%s: Element <%s> unknown - ignoring.",
            m_strXmlFileName.c_str(), strElem.c_str());
      LOGWARN("%s", m_bufErrMsg);
    }

    /*!
     * \brief Basic sanity checks on parsed parameters.
     *
     * \param [in] params   Pan-Tilt calibration parameters.
     *
     * \copydoc doc_return_std
     */
    int validate(PanTiltCalibParams &params);

    /*!
     * \brief Convert text to integer.
     *
     * \param strElem   Enclosing element name.
     * \param strText   Element text to convert.
     * \param [out] val Converted integer value.
     *
     * \copydoc doc_return_std
     */
    int strToInt(const std::string &strElem,
                 const std::string &strText,
                 int               &val);
  };

} // pan_tilt namespace

#endif // _PT_XML_CALIB_H
