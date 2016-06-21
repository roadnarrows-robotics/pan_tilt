////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptXmlCalib.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltXmlCalib - The Pan-Tilt calibration data class implimentation.
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

#include <stdio.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/appkit/Xml.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"
#include "pan_tilt/ptXmlCalib.h"

using namespace std;
using namespace rnr;
using namespace pan_tilt;

int PanTiltXmlCalib::load(PanTiltCalibParams &params,
                          const string       &strSearchPath,
                          const string       &strXmlFileName,
                          bool               bAllInstances)
{
  vector<string>  vecPath;          // vector of search paths
  string          fqname;           // fully qualified file name
  bool            bFoundInstance;   // [not] found instance
  size_t          i;                // working index
  int             rc;               // return code

  params.m_bIsSpecified = false;

  split(strSearchPath, ':', vecPath);

  bFoundInstance  = false;
  rc              = PT_OK;

  for(i=0; i<vecPath.size(); ++i)
  {
    fqname = expandFile(vecPath[i], strXmlFileName);

    if( access(fqname.c_str(), F_OK) == 0 )
    {
      LOGDIAG2("Loading Pan-Tilt description XML file: %s.", fqname.c_str());

      bFoundInstance = true;

      if( (rc = Xml::loadFile(fqname)) < 0 )
      {
        LOGERROR("Parse of Pan-Tilt description from XML file %s failed.",
            fqname.c_str());
        rc = -PT_ECODE_XML;
      }
      else
      {
        rc = setPanTiltParamsFromDOM(params);
      }

      if( rc == PT_OK )
      {
        LOGDIAG2("Pan-Tilt description from XML file %s loaded.",
            fqname.c_str());
      }

      if( !bAllInstances )
      {
        break;
      }
    }
  }

  if( !bFoundInstance )
  {
    LOGDIAG2("XML file %s not found.", strXmlFileName.c_str());
    rc = -PT_ECODE_XML;
  }

  if( rc == PT_OK )
  {
    rc = validate(params);
  }

  return rc;
}

int PanTiltXmlCalib::loadFile(const string &strXmlFileName)
{
  int   rc;

  rc = Xml::loadFile(strXmlFileName);

  return rc < 0? -PT_ECODE_XML: PT_OK;
}

int PanTiltXmlCalib::loadFile(PanTiltCalibParams &params,
                              const string       &strXmlFileName)
{
  int   rc;

  if( (rc = Xml::loadFile(strXmlFileName)) == OK )
  {
    rc = setPanTiltParamsFromDOM(params);
  }

  return rc < 0? -PT_ECODE_XML: PT_OK;
}

int PanTiltXmlCalib::saveFile(const string &strXmlFileName)
{
  int   rc;

  rc = Xml::saveFile(strXmlFileName);

  return rc < 0? -PT_ECODE_XML: PT_OK;
}

int PanTiltXmlCalib::saveFile(const PanTiltCalibParams &params,
                              const string             &strDirName,
                              const string             &strXmlFileName)
{
  string      strXmlOut;
  FILE       *fp;
  int         rc;

  strXmlOut = expandFile(strDirName, strXmlFileName);

  if( strXmlOut.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  m_strXmlFileName = strXmlOut;

  // open file
  if( (fp = fopen(m_strXmlFileName.c_str(), "w+")) == NULL )
  {
    setErrorMsg("%s: %s(errno=%d).",
        m_strXmlFileName.c_str(), strerror(errno), errno);
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  makeXmlHead();
  makeXmlTail();

  //
  // XML head.
  //
  fprintf(fp, "%s", m_strXmlHead.c_str());
  fprintf(fp, "<!-- RoadNarrows Pan-Tilt -->\n");

  //
  // Calibration major element.
  //
  fprintf(fp, "  <!-- Pan-Tilt Calibration -->\n");
  fprintf(fp, "  <%s>\n", m_strMajElemCalib.c_str());

  //
  // Pan joint section element.
  //
  fprintf(fp, "    <!-- Pan Joint -->\n");
  fprintf(fp, "    <%s>\n", m_strSecElemPan.c_str());

  fprintf(fp, "      <!-- encoder value at zero point -->\n");
  fprintf(fp, "      <%s>%d</%s>\n",
      m_strElemZero.c_str(), params.m_nPanEncZero, m_strElemZero.c_str());

  fprintf(fp, "      <!-- minimum odometer value -->\n");
  fprintf(fp, "      <%s>%d</%s>\n",
      m_strElemMin.c_str(), params.m_nPanOdMin, m_strElemMin.c_str());

  fprintf(fp, "      <!-- maximum odometer value -->\n");
  fprintf(fp, "      <%s>%d</%s>\n",
      m_strElemMax.c_str(), params.m_nPanOdMax, m_strElemMax.c_str());

  fprintf(fp, "    </%s>\n", m_strSecElemPan.c_str());

  //
  // Tilt joint section element.
  //
  fprintf(fp, "\n");
  fprintf(fp, "    <!-- Tilt Joint -->\n");
  fprintf(fp, "    <%s>\n", m_strSecElemTilt.c_str());

  fprintf(fp, "      <!-- encoder value at zero point -->\n");
  fprintf(fp, "      <%s>%d</%s>\n",
      m_strElemZero.c_str(), params.m_nTiltEncZero, m_strElemZero.c_str());

  fprintf(fp, "      <!-- minimum odometer value -->\n");
  fprintf(fp, "      <%s>%d</%s>\n",
      m_strElemMin.c_str(), params.m_nTiltOdMin, m_strElemMin.c_str());

  fprintf(fp, "      <!-- maximum odometer value -->\n");
  fprintf(fp, "      <%s>%d</%s>\n",
      m_strElemMax.c_str(), params.m_nTiltOdMax, m_strElemMax.c_str());

  fprintf(fp, "    </%s>\n", m_strSecElemTilt.c_str());

  fprintf(fp, "  </%s>\n\n", m_strMajElemCalib.c_str());

  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG2("Saved calibration to %s XML file.", m_strXmlFileName.c_str());

  return PT_OK;
}

int PanTiltXmlCalib::setPanTiltParamsFromDOM(PanTiltCalibParams &params)
{
  TiXmlElement *pElem1, *pElem2;
  const char   *sValue;
  int           rc;

  // root element
  if( m_pElemRoot == NULL )
  {
    setErrorMsg("Missing DOM and/or <%s> root element missing.",
       m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  // search for major elements
	for(pElem1 = m_pElemRoot->FirstChildElement(), rc = PT_OK;
      (pElem1 != NULL) && (rc == PT_OK);
      pElem1 = pElem1->NextSiblingElement())
  {
    if( (sValue = pElem1->Value()) == NULL )
    {
      continue;
    }

    //
    // Calibration major element. Walk through child elements and convert.
    //
    else if( !strcasecmp(sValue, m_strMajElemCalib.c_str()) )
    {
      // search for section elements
	    for(pElem2 = pElem1->FirstChildElement();
          (pElem2 != NULL) && (rc == PT_OK);
          pElem2 = pElem2->NextSiblingElement())
      {
        // child element name
        if( (sValue = pElem2->Value()) == NULL )
        {
          continue;
        }

        // pan joint calibration section
        else if( !strcasecmp(sValue, m_strSecElemPan.c_str()) )
        {
          rc = setPanParamsFromDOM(pElem2, params);
        }

        // tilt joint calibration section
        else if( !strcasecmp(sValue, m_strSecElemTilt.c_str()) )
        {
          rc = setTiltParamsFromDOM(pElem2, params);
        }

        // unknown
        else
        {
          warnUnknownElem(sValue);
        }
      }
    }
  }

  return rc;
}

int PanTiltXmlCalib::setDOMFromPanTiltParams(const PanTiltCalibParams &params)
{
  // TODO
  return -PT_ECODE_GEN;
}

int PanTiltXmlCalib::setPanParamsFromDOM(TiXmlElement       *pElemSec,
                                         PanTiltCalibParams &params)
{
  TiXmlElement *pElem;
  const char   *sValue;
  int           rc;

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = PT_OK;
      (pElem != NULL) && (rc == PT_OK);
      pElem = pElem->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // minimum encoder value
    else if( !strcasecmp(sValue, m_strElemMin.c_str()) )
    {
      rc = strToInt(sValue, elemText(pElem), params.m_nPanOdMin);
    }

    // zero point encoder value
    else if( !strcasecmp(sValue, m_strElemZero.c_str()) )
    {
      rc = strToInt(sValue, elemText(pElem), params.m_nPanEncZero);
    }

    // maximum encoder value
    else if( !strcasecmp(sValue, m_strElemMax.c_str()) )
    {
      rc = strToInt(sValue, elemText(pElem), params.m_nPanOdMax);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == PT_OK )
  {
    LOGDIAG3("%s: Pan joint calibration parameters parsed.",
      m_strXmlFileName.c_str());
  }

  return rc;
}

int PanTiltXmlCalib::setTiltParamsFromDOM(TiXmlElement       *pElemSec,
                                          PanTiltCalibParams &params)
{
  TiXmlElement *pElem;
  const char   *sValue;
  int           rc;

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = PT_OK;
      (pElem != NULL) && (rc == PT_OK);
      pElem = pElem->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // minimum encoder value
    else if( !strcasecmp(sValue, m_strElemMin.c_str()) )
    {
      rc = strToInt(sValue, elemText(pElem), params.m_nTiltOdMin);
    }

    // zero point encoder value
    else if( !strcasecmp(sValue, m_strElemZero.c_str()) )
    {
      rc = strToInt(sValue, elemText(pElem), params.m_nTiltEncZero);
    }

    // maximum encoder value
    else if( !strcasecmp(sValue, m_strElemMax.c_str()) )
    {
      rc = strToInt(sValue, elemText(pElem), params.m_nTiltOdMax);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == PT_OK )
  {
    LOGDIAG3("%s: Pan joint calibration parameters parsed.",
      m_strXmlFileName.c_str());
  }

  return rc;
}

int PanTiltXmlCalib::validate(PanTiltCalibParams &params)
{
  //
  // Pan
  //
  if( params.m_nPanEncZero == PanTiltCalibParams::UNSPEC )
  {
    setErrorMsg("%s: Element <%s><%s> parameter unspecified.",
      m_strXmlFileName.c_str(), m_strSecElemPan.c_str(), m_strElemZero.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  if( params.m_nPanOdMin == PanTiltCalibParams::UNSPEC )
  {
    setErrorMsg("%s: Element <%s><%s> parameter unspecified.",
      m_strXmlFileName.c_str(), m_strSecElemPan.c_str(), m_strElemMin.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  if( params.m_nPanOdMax == PanTiltCalibParams::UNSPEC )
  {
    setErrorMsg("%s: Element <%s><%s> parameter unspecified.",
      m_strXmlFileName.c_str(), m_strSecElemPan.c_str(), m_strElemMax.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  //
  // Tilt
  //
  if( params.m_nTiltEncZero == PanTiltCalibParams::UNSPEC )
  {
    setErrorMsg("%s: Element <%s><%s> parameter unspecified.",
     m_strXmlFileName.c_str(), m_strSecElemTilt.c_str(), m_strElemZero.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  if( params.m_nTiltOdMin == PanTiltCalibParams::UNSPEC )
  {
    setErrorMsg("%s: Element <%s><%s> parameter unspecified.",
     m_strXmlFileName.c_str(), m_strSecElemTilt.c_str(), m_strElemMin.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  if( params.m_nTiltOdMax == PanTiltCalibParams::UNSPEC )
  {
    setErrorMsg("%s: Element <%s><%s> parameter unspecified.",
      m_strXmlFileName.c_str(), m_strSecElemTilt.c_str(), m_strElemMax.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }

  params.m_bIsSpecified = true;

  return PT_OK;
}

int PanTiltXmlCalib::strToInt(const string &strElem,
                              const string &strText,
                              int          &val)
{
  if( Xml::strToInt(strText, val) == OK )
  {
    return PT_OK;
  }
  else
  {
    setErrorMsg("%s: Element <%s> text \"%s\": Not a number.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -PT_ECODE_XML;
  }
}
