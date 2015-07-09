////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptUtils.cxx
//
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Pan-Tilt utilities.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2014-2015  RoadNarrows
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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <wordexp.h>
#include <errno.h>

#include <string>
#include <sstream>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"

using namespace std;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

/*!
 * \ingroup libpan_tilt
 * \brief Pan-Tilt Error Code String Table.
 *
 * Table is indexed by pan-tilt error codes (see \ref pt_ecodes). Keep
 * in sync.
 */
static const char *EcodeStrTbl[] =
{
  "Ok",                                     ///< [PT_OK]

  "Error",                                  ///< [PT_ECODE_GEN]
  "System error",                           ///< [PT_ECODE_SYS]
  "Internal error",                         ///< [PT_ECODE_INTERNAL]
  "Bad value",                              ///< [PT_ECODE_BAD_VAL]
  "Too big",                                ///< [PT_ECODE_TOO_BIG]
  "Too small",                              ///< [PT_ECODE_TOO_SMALL]
  "Value out-of-range",                     ///< [PT_ECODE_RANGE]
  "Invalid operation",                      ///< [PT_ECODE_BAD_OP]
  "Operation timed out",                    ///< [PT_ECODE_TIMEDOUT]
  "Device not found",                       ///< [PT_ECODE_NO_DEV]
  "No resource available",                  ///< [PT_ECODE_NO_RSRC]
  "Resource busy",                          ///< [PT_ECODE_BUSY]
  "Cannot execute",                         ///< [PT_ECODE_NO_EXEC]
  "Permissions denied",                     ///< [PT_ECODE_PERM]
  "Dynamixel chain or servo error",         ///< [PT_ECODE_DYNA]
  "Bad format",                             ///< [PT_ECODE_FORMAT]
  "File not found",                         ///< [PT_ECODE_NO_FILE]
  "XML error",                              ///< [PT_ECODE_XML]
  "Robot is in an alarmed state",           ///< [PT_ECODE_ALARMED]
  "Operation interrupted",                  ///< [PT_ECODE_INTR]
  "Robotic joint(s) movement obstructed",   ///< [PT_ECODE_COLLISION]
  "Robot emergency stopped",                ///< [PT_ECODE_ESTOP]

  "Invalid error code"                      ///< [PT_ECODE_BADEC]
};


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

const char *pan_tilt::getStrError(const int ecode)
{
  int ec = ecode >= 0 ? ecode : -ecode;

  if( ec >= arraysize(EcodeStrTbl) )
  {
    ec = PT_ECODE_BADEC;
  }

  return EcodeStrTbl[ec];
}

uint_t pan_tilt::strToVersion(const string &str)
{
  int   nMajor = 0;
  int   nMinor = 0;
  int   nRevision = 0;

  sscanf(str.c_str(), "%d.%d.%d", &nMajor, &nMinor, &nRevision);

  return PT_VERSION(nMajor, nMinor, nRevision);
}

vector<string> &pan_tilt::split(const string   &s,
                                 char           delim,
                                 vector<string> &elems)
{
  stringstream ss(s);
  string item;

  while( getline(ss, item, delim) )
  {
    elems.push_back(item);
  }
  return elems;
}

vector<string> pan_tilt::split(const string &s, char delim)
{
  vector<string> elems;

  split(s, delim, elems);

  return elems;
}

string pan_tilt::expandFile(const string &strFileName)
{
  wordexp_t   exp_result;
  string      strExpName;

  if( strFileName.empty() )
  {
    strExpName = strFileName;
  }
  else
  {
    wordexp(strFileName.c_str(), &exp_result, 0);

    strExpName = exp_result.we_wordv[0];

    wordfree(&exp_result);
  }

  return strExpName;
}

string pan_tilt::expandFile(const string &strDirName, const string &strFileName)
{
  wordexp_t   exp_result;

  if( strDirName.empty() )
  {
    return expandFile(strFileName);
  }
  else
  {
    return expandFile(strDirName + '/' + strFileName);
  }
}
