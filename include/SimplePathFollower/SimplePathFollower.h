// -*- C++ -*-
/*!
 * @file  SimplePathFollower.h
 * @brief Simple Algorithm Path Follower
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SIMPLEPATHFOLLOWER_H
#define SIMPLEPATHFOLLOWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "MobileRobotSVC_impl.h"


#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

#include "SimpleFollower.h"


enum SIMPLE_PATH_FOLLOWER_MODE {
  MODE_NORMAL,
  MODE_TIMEOUT,
  MODE_OUTOFRANGE,
  MODE_GOALED,
};


/*!
 * @class SimplePathFollower
 * @brief Simple Algorithm Path Follower
 *
 */
class SimplePathFollower
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  SimplePathFollower(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~SimplePathFollower();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 0
   */
  int m_debug;
  /*!
   * 
   * - Name:  directionGain
   * - DefaultValue: 1.0
   */
  float m_distanceToTranslationGain;
  /*!
   * 
   * - Name:  distanceGain
   * - DefaultValue: 1.0
   */
  float m_distanceToRotationGain;

  /*!
   * 
   * - Name:  directionGain
   * - DefaultValue: 1.0
   */
  float m_directionToTranslationGain;
  /*!
   * 
   * - Name:  distanceGain
   * - DefaultValue: 1.0
   */
  float m_directionToRotationGain;
  /*!
   * 
   * - Name:  directionGain
   * - DefaultValue: 1.0
   */
  float m_approachDistanceGain;
  /*!
   * 
   * - Name:  distanceGain
   * - DefaultValue: 1.0
   */
  float m_approachDirectionGain;
  /*!
   * 
   * - Name:  directionGain
   * - DefaultValue: 1.0
   */
  float m_maxVelocity;
  /*!
   * 
   * - Name:  distanceGain
   * - DefaultValue: 1.0
   */
  float m_minVelocity;
  /*!
   *
   *
   */
  float m_poseTimeout;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::Path2D m_path;
  /*!
   */
  InPort<RTC::Path2D> m_pathIn;
  RTC::TimedPose2D m_currentPose;
  /*!
   */
  InPort<RTC::TimedPose2D> m_currentPoseIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedVelocity2D m_velocity;
  /*!
   */
  OutPort<RTC::TimedVelocity2D> m_velocityOut;
  
  enum __path_follower_state {
    STATE_HALT,
    STATE_FOLLOWING,
    STATE_OUTOFRANGE,
    STATE_ERROR,
  };
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_PathFollowerPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  PathFollowerSVC_impl m_pathFollower;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

  bool m_poseUpdated;
  SimpleFollower m_pathFollowerObj;

public:

  void startFollow() {
    m_pathFollowerObj.startFollow(m_path);
  }

  void stopFollow() {
	  m_pathFollowerObj.stopFollow();
  }

  void setPath(const RTC::Path2D& path) {
    this->m_path = path;
  }

  coil::TimeValue m_lastReceivedTime;

  SIMPLE_PATH_FOLLOWER_MODE m_Mode;

public:
  SIMPLE_PATH_FOLLOWER_MODE getMode() {return m_Mode;}

  bool isGoal() {
    return m_pathFollowerObj.isGoal();
  }
};


extern "C"
{
  DLL_EXPORT void SimplePathFollowerInit(RTC::Manager* manager);
};

#endif // SIMPLEPATHFOLLOWER_H
