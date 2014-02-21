###############################################################################
#
#
# Package:   RoadNarrows Robotics ROS Pan-Tilt Robot Package
#
# Link:      https://github.com/roadnarrows-robotics/pan_tilt
#
# ROS Node:  pan_tilt_*
#
# File:      Utils.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Utilities.
##
## \author Daniel Packard (daniel@roadnarrows.com)
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2014.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import math

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from PIL import Image, ImageTk

# ------------------------------------------------------------------------------
# Utilities
# ------------------------------------------------------------------------------

#
## \brief Load icon image from file name.
##
## \param filename    Icon file name.
##
## \return Returns icon widget on success, None on failure.
#
def loadIcon(filename, imagePath='.:/usr/local/icons/pan_tilt'):
  # no file name
  if filename is None or len(filename) == 0:
    return None;
  # absolute file name
  if filename[0] == os.path.sep:
    try:
      return ImageTk.PhotoImage(Image.open(filename))
    except IOError:
      return None
  # relative file name - search path for file
  for path in imagePath:
    fqname = path + os.path.sep + filename
    try:
      return ImageTk.PhotoImage(Image.open(fqname))
    except IOError:
      continue
  return None

#
## Round to nearest 100th.
#
def round100th(x):
  return math.floor((x + 0.005) * 100.0) / 100.0

#
## Round to nearest 10th.
#
def round10th(x):
  return math.floor((x + 0.05) * 10.0) / 10.0

#
## Degrees to radians.
#
def degToRad(deg):
  return deg / 180.0 * math.pi

#
## Radians to degrees.
#
def radToDeg(rad):
  return rad / math.pi * 180.0



