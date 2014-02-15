////////////////////////////////////////////////////////////////////////////////
//
// Package:   pan_tilt
//
// Library:   libpan_tilt
//
// File:      ptRobot.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief PanTiltRobot - the pan-tilt robot class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#ifndef _PAN_TILT_ROBOT_H
#define _PAN_TILT_ROBOT_H

#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"

#include <string>
#include <map>

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "pan_tilt/pan_tilt.h"
#include "pan_tilt/ptUtils.h"
#include "pan_tilt/ptDesc.h"
#include "pan_tilt/ptJoint.h"
#include "pan_tilt/ptCalib.h"
#include "pan_tilt/ptTraj.h"
#include "pan_tilt/ptStatus.h"

namespace pan_tilt
{
  /*!
   * \brief Pan-Tilt robotic mechanism class.
   */
  class PanTiltRobot
  { 
  public:
    /*!
     * \brief Map of robot joints.
     *
     * \termblock
     * \term key: \termdata master servo id \endterm
     * \term mapped type: \termdata joint data \endterm
     * \endtermblock
     *
     * \note Joint order is critical. Ascending servo ids keeps map in order,
     * but if this cannot be guaranteed, then change strategy.
     */
    typedef map<int, PanTiltRobotJoint> MapRobotJoints;

    /*!
     * \brief Indirect map of robot joints.
     *
     * \termblock
     * \term key: \termdata joint name \endterm
     * \term mapped type: \termdata master servo id \endterm
     * \endtermblock
     */
    typedef map<std::string, int> IMapRobotJoints;

    /*!
     * \brief Asynchronous task id.
     */
    enum AsyncTaskId
    {
      AsyncTaskIdNone,        ///< no task
      AsyncTaskIdCalibrate    ///< calibrate pan-tilt task id

      // add others here, as needed
    };

    /*!
     * \brief Default constructor.
     */
    PanTiltRobot();

    /*!
     * \brief Destructor.
     */
    virtual ~PanTiltRobot();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Communication and Robot Initialization Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to pan-tilt.
     *
     * \param strSerDevName   Serial device name connected to Dynamixel bus.
     * \param nBaudRate       Serial baud rate.
     *
     * \copydoc doc_return_std
     */
    int connect(const std::string &strSerDevName="/dev/ttyUSB0",
                const int          nBaudRate=1000000);

    /*!
     * \brief Disconnect from pan-tilt.
     *
     * \copydoc doc_return_std
     */
    int disconnect();

    /*!
     * \brief Calibrate pan-tilt's odometers and limit switch positions.
     *
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     *
     * \copydoc doc_return_std
     */
    int calibrate(bool bForceRecalib=true);

    /*!
     * \brief Asynchronously calibrate pan-tilt's odometers and limit switch
     * positions.
     *
     * Call \ref cancelAsync() to cancel operation.
     *
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     *
     * \copydoc doc_return_std
     */
    int calibrateAsync(bool bForceRecalib=true)
    {
      if( m_eAsyncTaskState != PanTiltAsyncTaskStateIdle )
      {
        LOGERROR("Already executing asynchronous task.");
        return -PT_ECODE_NO_RSRC;
      }
      else
      {
        m_eAsyncTaskId  = AsyncTaskIdCalibrate;
        m_pAsyncTaskArg = (void *)bForceRecalib;

        return createAsyncThread();
      }
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Canned Movements
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Move pan-tilt to its pre-defined balanced position.
     *
     * \par Some Considerations:
     * This move assumes an unrestricted workspace save for an horizontal plane
     * defined by the platform on which the pan-tilt is mounted to.
     *
     * \copydoc doc_return_std
     */
    int gotoZeroPtPos();

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Fundamental Pan-Tilt Operations
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Emergency stop.
     *
     * All servos will stop driving, so pan-tilt or accessories may fall.
     *
     * \copydoc doc_return_std
     */
    int estop();

    /*!
     * \brief Freeze pan-tilt at current position.
     *
     * Servos are still being driven.
     *
     * \copydoc doc_return_std
     */
    int freeze();

    /*!
     * \brief Release pan-tilt.
     *
     * Servos will stop driving, so the pan-til may fall.
     * Typically, the pan-tilt is released during manual teaching.
     *
     * \copydoc doc_return_std
     */
    int release();

    /*!
     * \brief Attempt to clear all servo alarms in all kinematic chains.
     *
     * \note Not all alarms are clearable (e.g. temperature).
     *
     * \copydoc doc_return_std
     */
    int clearAlarms();

    /*!
     * \brief Reset (clears) emergency stop condition.
     *
     * \note Servos are not re-powered until an move or freeze action is called.
     */
    void resetEStop()
    {
      m_bIsEStopped = false;
    }

    /*!
     * \brief Move pan-tilt through trajectory point.
     *
     * \param trajectoryPoint   Trajectory end point.
     *
     * \copydoc doc_return_std
     */
    int move(PanTiltJointTrajectoryPoint &trajectoryPoint);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Pan-Tilt State and Status
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the robot current status.
     *
     * \param [in,out] robotStatus   Robot status.
     *
     * \copydoc doc_return_std
     */
    int getRobotStatus(PanTiltRobotStatus &robotStatus);

    /*!
     * \brief Get the joint states of a kinematic chain.
     *
     * \param [in,out] jointStatePoint  Joint state point.
     *
     * \copydoc doc_return_std
     */
    int getJointState(PanTiltJointStatePoint &jointStatePoint);

    /*!
     * \brief Get trajectory feedback.
     *
     * \param [in,out] trajectoryFeedback  Joint state point.
     *
     * \copydoc doc_return_std
     */
    int getTrajectoryState(PanTiltJointTrajectoryFeedback &trajectoryFeedback);

    /*!
     * \brief Set robot's operational mode.
     *
     * \return eRobotMode Robot operation mode. See \ref PanTiltRobotMode.
     */
    void setRobotMode(PanTiltRobotMode eRobotMode)
    {
      m_eRobotMode = eRobotMode;
    }

    /*!
    /*!
     * \brief Test if any joint in any of the kinematic chains is moving.
     *
     * \return Returns true or false.
     */
    bool isInMotion();

    /*!
     * \brief Cancel any asynchronous task.
     *
     * \note There may be a little delay between canceling an async task
     * and the task actually stopping.
     */
    void cancelAsyncTask();

    /*!
     * \brief Get the current asynchronous task state.
     *
     * \return \ref PanTiltAsyncTaskState enum value.
     */
    PanTiltAsyncTaskState getAsyncState()
    {
      return m_eAsyncTaskState;
    }

    /*!
     * \brief Get the last asynchronous task return code.
     *
     * \return
     * Returns PT_OK when task terminated successfully.
     * Otherwise, returns \h_lt 0 \ref pt_ecodes.
     */
    int getAsyncRc()
    {
      return m_rcAsyncTask;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if connected to pan-tilt hardware.
     *
     * \return Returns true or false.
     */
    bool isConnected()
    {
      return (m_pDynaComm != NULL) && m_pDynaComm->IsOpen()? true: false;
    }

    /*!
     * \brief Get the pan-tilt robot hardware version number.
     *
     * \param [out] nVerMajor     Major version number.
     * \param [out] nVerMinor     Minor version number.
     * \param [out] nVerRevision  Revision version number.
     */
    void getVersion(int &nVerMajor, int &nVerMinor, int &nRevision)
    {
      uint_t  uHwVer;

      uHwVer    = strToVersion(m_descPanTilt.getProdHwVer());
      nVerMajor = PT_VER_MAJOR(uHwVer);
      nVerMinor = PT_VER_MINOR(uHwVer);
      nRevision = PT_VER_REV(uHwVer);
    }

    /*!
     * \brief Get the pan-tilt robot hardware version string.
     *
     * Version number strings are of the dotted form maj.min.rev.
     *
     * \return Version string.
     */
    std::string getVersion()
    {
      return m_descPanTilt.getProdHwVer();
    }

    /*!
     * \brief Get the pan-tilt product description.
     *
     * \return Returns pointer to description.
     */
    PanTiltDesc *getPanTiltDesc()
    {
      return &m_descPanTilt;
    }

    /*!
     * \brief Convenience function to get this pan-tilt description's base
     * product id.
     *
     * \return Returns product id. See \ref PanTiltProdId.
     */
    int getProdId()
    {
      return m_descPanTilt.getProdId();
    }
  
    /*!
     * \brief Convenience function to get this pan-tilt description's base
     * product name.
     *
     * \return Returns product name. See \ref PanTiltProdName.
     */
    std::string getProdName()
    {
      return m_descPanTilt.getProdName();
    }
  
    /*!
     * \brief Get the pan-tilt full brief descirption.
     *
     * \return Returns product brief description.
     */
    std::string getFullProdBrief()
    {
      return m_descPanTilt.getFullProdBrief();
    }
  
    /*!
     * \brief Test if robot is fully described via configuration XML.
     *
     * \return Returns true or false.
     */
    int isDescribed()
    {
      return m_descPanTilt.isDescribed();
    }

    /*!
     * \brief Test if robot is calibrated.
     *
     * \return Returns true or false.
     */
    int isCalibrated()
    {
      return m_eOpState == PanTiltOpStateCalibrated;
    }

    /*!
     * \brief Test if robot is current emergency stopped.
     *
     * \return Returns true or false.
     */
    int isEStopped()
    {
      return m_bIsEStopped;
    }

    /*!
     * \brief Test if robot servos are currently being driven (powered).
     *
     * \return Returns true or false.
     */
    int areServosPowered()
    {
      return m_bAreServosPowered;
    }

    /*!
     * \brief Test if robot is alarmed.
     *
     * \return Returns true or false.
     */
    int isAlarmed()
    {
      return m_bAlarmState;
    }

    /*!
     * \brief Get the dynamixel chain object.
     *
     * \return Returns point to DynaChain object.
     */
    DynaChain *getDynaChain()
    {
      return m_pDynaChain;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Kinematic Access and Mapping Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get pan-tilt joint in kinematic chain at the given index.
     *
     * \param index   Joint index.
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    PanTiltRobotJoint *getJointAt(int index)
    {
      return index < m_kin.size()? &m_kin.at(index): NULL;
    }

    /*!
     * \brief Get pan-tilt joint in kinematic chain.
     *
     * \param strName   Joint name (key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    PanTiltRobotJoint *getJoint(std::string &strName)
    {
      IMapRobotJoints::iterator  iter;

      if( (iter = m_imapKin.find(strName)) != m_imapKin.end() )
      {
        return &(m_kin[iter->second]);
      }
      else
      {
        return NULL;
      }
    }

    /*!
     * \brief Get robotic joint in kinematic chain.
     *
     * \param nServoId  Master servo id (primary key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    PanTiltRobotJoint *getJoint(int nServoId)
    {
      MapRobotJoints::iterator  iter;

      if( (iter = m_kin.find(nServoId)) != m_kin.end() )
      {
        return &(iter->second);
      }
      else
      {
        return NULL;
      }
    }

    /*!
     * \brief Get robotic joint's master servo.
     *
     * \param joint     Robotic joint.
     *
     * \return If found, returns pointer to servo. Otherwise returns NULL.
     */
    DynaServo *getMasterServo(PanTiltRobotJoint &joint)
    {
      return m_pDynaChain!=NULL? m_pDynaChain->GetServo(joint.m_nMasterServoId):
                                 NULL;
    }

    /*!
     * \brief Convert joint position to servo odometer position.
     *
     * Supported input position units:
     * \termblock
     * \term units_radians \termdata radians \endterm
     * \term units_degrees \termdata degrees \endterm
     * \term units_raw \termdata servo raw odometer ticks \endterm
     * \endtermblock
     *
     * \param joint     Robotic joint.
     * \param fPosition Joint position.
     * \param units     Input position units.
     *
     * \return Odometer ticks.
     */
    int jointPositionToOdPos(PanTiltRobotJoint &joint,
                             double         fPosition,
                             units_t        units=units_radians);


    /*!
     * \brief Convert joint velocity to servo raw speed.
     *
     * Out of range values are capped at servo [min,max] values.
     *
     * Supported input velocicy units:
     * \termblock
     * \term units_norm \termdata normalized [0.0, 1.0] of maximum \endterm
     * \term units_percent \termdata percent [0.0, 100.0] of maximum \endterm
     * \term units_permil \termdata permil [0.0, 1000.0] of maximum \endterm
     * \term units_rad_per_s \termdata radians per second \endterm
     * \term units_raw \termdata servo raw speed units \endterm
     * \endtermblock
     *
     * \param joint       Robotic joint.
     * \param nOdGoalPos  Goal odometer position (required to determine
     *                    direction).
     * \param fVelocity   Joint velocity.
     * \param units       Input velocity units.
     *
     * \return Raw servo speed units.
     */
    int jointVelocityToRawSpeed(PanTiltRobotJoint &joint,
                                int                nOdGoalPos,
                                double             fVelocity,
                                units_t            units=units_percent);

    /*!
     * \brief Get the current joint position.
     *
     * Some joint positions odometer readings are effected by other joint
     * positions. Namely, the wrist pitch affects wrist rotation.
     * Adjustments are made for any of these cross-related effects.
     *
     * \param joint     Robotic joint.
     * \param units     Output position units. See \ref jointPositionToOdPos.
     *
     * \return Joint position in the given units.
     */
    double getCurJointPosition(PanTiltRobotJoint &joint,
                               units_t            units=units_radians);

    /*!
     * \brief Convert servo odometer position to joint position.
     *
     * \param joint     Robotic joint.
     * \param nOdPos    Odometer position (ticks).
     * \param units     Output position units. See \ref jointPositionToOdPos.
     *
     * \return Joint position in the given units.
     */
    double odPosToJointPosition(PanTiltRobotJoint &joint,
                                int            nOdPos,
                                units_t        units=units_radians);

    /*!
     * \brief Get the current joint velocity.
     *
     * Some joint velocities are effected by other joint movements.
     * Namely, the wrist pitch affects wrist rotation.
     * Adjustments are made for any of these cross-related effects.
     *
     * \param joint     Robotic joint.
     * \param units     Output velocity units. See \ref jointVelocityToRawSpeed.
     *
     * \return Joint velocity in the given units.
     */
    double getCurJointVelocity(PanTiltRobotJoint &joint,
                               units_t            units=units_percent);

    /*!
     * \brief Convert servo raw speed to joint velocity.
     *
     * \param joint     Robotic joint.
     * \param nSpeed    Servo speed (unitless).
     * \param units     Output velocity units. See \ref jointVelocityToRawSpeed.
     *
     * \return Joint velocity in the given units.
     */
    double rawSpeedToJointVelocity(PanTiltRobotJoint &joint,
                                   int                nOdPos,
                                   units_t            units=units_percent);

  protected:
    // state
    PanTiltDesc       m_descPanTilt;    ///< pan-tilt description
    PanTiltRobotMode  m_eRobotMode;     ///< robot operating mode
    PanTiltOpState    m_eOpState;       ///< pan-tilt operational state
    bool              m_bIsEStopped;    ///< pan-tilt is [not] emergency stopped
    bool              m_bAlarmState;    ///< robot is [not] alarmed
    bool              m_bAreServosPowered;///< pan-tilt servos are [not] driven

    // dynamixel i/f
    DynaComm         *m_pDynaComm;      ///< dynamixel communication
    DynaChain        *m_pDynaChain;     ///< dynamixel chain
    DynaBgThread     *m_pDynaBgThread;  ///< dynamixel background thread

    // joints and kinematics chains
    MapRobotJoints    m_kin;            ///< robot kinematic chain
    IMapRobotJoints   m_imapKin;        ///< robot indirect kinematic map

    // motion
    PanTiltJointTrajectoryPoint m_lastTraj; ///< last trajectory point

    // asynchronous task control
    PanTiltAsyncTaskState m_eAsyncTaskState;  ///< asynchronous task state
    int               m_rcAsyncTask;      ///< last async task return code
    AsyncTaskId       m_eAsyncTaskId;     ///< asynchronous task id
    void             *m_pAsyncTaskArg;    ///< asynchronous argument
    pthread_t         m_threadAsync;      ///< async pthread identifier 

    friend class PanTiltCalib;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Scan Hardware Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Scan pan-tilt hardware.
     *
     * \copydoc doc_return_std
     */
    int scan();

    /*!
     * \brief Scan pan-tilt dynamixel bus hardware, determine product, and
     * validate..
     *
     * \param nMaxTries   Maximums number of scan attempts before failing.
     *
     * \copydoc doc_return_std
     */
    int scanDynaBus(int nMaxTries);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Convert Static Specifications Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert specification to operational parameters.
     *
     * The specifications must be defined and the connecton to the pan-tilt
     * hardware must be open.
     *
     * There in 1 kinematic chain supported by the pan-tilt library.
     *
     * \note A kinematic chain differs from the dynamixel chain. Pan-tilt
     * supports one dynamixel bus in which all servos hang. A dynamixel chain
     * defines this communication bus. Servos on the bus can be associated with
     * the pan-tilt kinematic chain, or user-defined chains.
     *
     * \copydoc doc_return_std
     */
    int convertSpec();

    /*!
     * \brief Add a joint to robot's kinematic chain.
     *
     * \param [in] pSpecJoint   Pointer to joint spcecification.
     * \param [in] pSpecServo   Pointer to master servo spcecification.
     * \param [out] kin         Modified kinematics chain of joints.
     * \param [out] imap        Indirect map of kinematic chain.
     *
     * \copydoc doc_return_std
     */
    int addRobotJoint(PanTiltSpecJoint_T  *pSpecJoint,
                      PanTiltSpecServo_T  *pSpecServo,
                      MapRobotJoints  &kin,
                      IMapRobotJoints &imap);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Hardware and Software Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Faux calibrate pan-tilt.
     *
     * \par DANGER:
     * This function should in place of calibrate() for debugging.
     * Make suere the pan-tilt is placed at the zero point prior to starting.
     */
    void fauxcalibrate();

    /*!
     * \brief Configure pan-tilt for calibration.
     *
     * The pan-tilt effectors are placed in a safe,
     * albeit slow, operational mode prior to actual moves performed during
     * calibration.
     *
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     *
     * \copydoc doc_return_std
     */
    int configForCalibration(bool bForceRecalib);

    /*!
     * \brief Configure pan-tilt for normal operation.
     *
     * The pan-tilt effectors are configured for normal operation.
     *
     * \copydoc doc_return_std
     */
    int configForOperation();

    /*!
     * \brief Configure the EEPROM for all pan-tilt servos in kinematic chain.
     *
     * For each servo, ensure that the proper servo EEPROM values are set.
     *
     * \param kin   Kinematic chain of joints.
     *
     * \copydoc doc_return_std
     */
    int configEEPROMForAllServos(MapRobotJoints &kin);

    /*!
     * \brief Configure one pan-tilt servo EEPROM.
     *
     * \note System reads, writes, and sleeps are pthread cancelation points.
     *
     * \param nServoId    Servo id.
     * \param joint       Associated robotic joint.
     *
     * \copydoc doc_return_std
     */
    int configEEPROMForServo(int nServoId, PanTiltRobotJoint &joint);

    /*!
     * \brief Configure the RAM for all pan-tilt servos in kinematic chain.
     *
     * For each servo, ensure that the proper servo RAM values are set.
     *
     * \param kin   Kinematic chain of joints.
     *
     * \copydoc doc_return_std
     */
    int configRAMForAllServos(MapRobotJoints &kin);

    /*!
     * \brief Configure one pan-tilt servo RAM.
     *
     * \note System reads, writes, and sleeps are pthread cancelation points.
     *
     * \param nServoId    Servo id.
     * \param joint       Associated robotic joint.
     *
     * \copydoc doc_return_std
     */
    int configRAMForServo(int nServoId, PanTiltRobotJoint &joint);

    /*!
     * \brief Configure the RAM for all pan-tilt servos in kinematic chain.
     *
     * For each servo, ensure that the proper servo RAM values are set.
     *
     * \param kin             Kinematic chain of joints.
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     *
     * \copydoc doc_return_std
     */
    int configStateForAllServos(MapRobotJoints &kin, bool bForceRecalib);

    /*!
     * \brief Configure one pan-tilt servo RAM.
     *
     * \param nServoId        Servo id.
     * \param bIsMaster       Servo is [not] the master servo.
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     * \param joint           Associated robotic joint.
     *
     * \copydoc doc_return_std
     */
    int configStateForServo(int           nServoId,
                            bool          bIsMaster,
                            bool          bForceRecalib,
                            PanTiltRobotJoint &joint);

    /*!
     * \brief Configure soft over torque thresholds for all servos.
     *
     * This function overrides any specification value(s).
     *
     * \param kin             Kinematic chain of joints.
     * \param fOverTorqueTh   Over torque threshold (%).
     * \param fClearTorqueTh  Clear over torque condition threshold (%).
     *
     * \copydoc doc_return_std
     */
    int configSoftTorqueForAllServos(MapRobotJoints &kin,
                                     double         fOverTorqueTh,
                                     double         fClearTorqueTh);

    /*!
     * \brief Reset all odometers for pan-tilt servos in kinematic chain at
     * current servo positions.
     *
     * \param kin   Kinematic chain of joints.
     *
     * \copydoc doc_return_std
     */
    int resetOdometersForAllServos(MapRobotJoints &kin);

    /*!
     * \brief Reset odometer for one servo to the current position.
     *
     * \param nServoId    Servo id.
     * \param bIsReverse  Odometer positive direction is [not] reversed from
     *                    encoder.
     *
     * \copydoc doc_return_std
     */
    int resetOdometerForServo(int nServoId, bool bIsReverse);

    /*!
     * \brief Determine robot operational state from collective joint
     * operational states.
     *
     * \return Determined robot operational state synchronized with joint
     * states.
     */
    PanTiltOpState detRobotOpState();

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Asynchronous Operation Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Create the asynchronous thread.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    int createAsyncThread();

    /*!
     * \brief Asynchronous operation thread.
     *
     * \param pArg   Thread argument (point to PanTiltRobot object).
     * 
     * \return Returns NULL on thread exit.
     */
     static void *asyncThread(void *pArg);
  };

} // namespace pan_tilt


#endif // _PAN_TILT_ROBOT_H
