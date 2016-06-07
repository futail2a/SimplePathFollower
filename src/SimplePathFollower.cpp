// -*- C++ -*-
/*!
* @file  SimplePathFollower.cpp
* @brief Simple Algorithm Path Follower
* @date $Date$
*
* $Id$
*/

#include <stdio.h>
#include "SimplePathFollower.h"

// Module specification
// <rtc-template block="module_spec">
static const char* simplepathfollower_spec[] =
{
	"implementation_id", "SimplePathFollower",
	"type_name",         "SimplePathFollower",
	"description",       "Simple Algorithm Path Follower",
	"version",           "1.0.0",
	"vendor",            "Sugar Sweet Robotics",
	"category",          "Navigation",
	"activity_type",     "PERIODIC",
	"kind",              "DataFlowComponent",
	"max_instance",      "1",
	"language",          "C++",
	"lang_type",         "compile",
	// Configuration variables
	"conf.default.debug", "0",
	"conf.default.directionToTranslationGain", "0.0",
	"conf.default.distanceToTranslationGain", "0.0",
	"conf.default.directionToRotationGain", "0.0",
	"conf.default.distanceToRotationGain", "2.0",
	"conf.default.approachDistanceGain", "0.5",
	"conf.default.approachDirectionGain", "1.0",
	"conf.default.maxVelocity", "0.8",
	"conf.default.minVelocity", "0.2",
	"conf.default.poseTimeout", "3.0",
	// Widget
	"conf.__widget__.debug", "text",
	"conf.__widget__.directionGain", "text",
	"conf.__widget__.distanceGain", "text",
	// Constraints
	""
};
// </rtc-template>

/*!
* @brief constructor
* @param manager Maneger Object
*/
SimplePathFollower::SimplePathFollower(RTC::Manager* manager)
	// <rtc-template block="initializer">
	: RTC::DataFlowComponentBase(manager),
	m_pathIn("path", m_path),
	m_currentPoseIn("currentPose", m_currentPose),
	m_velocityOut("velocity", m_velocity)
	,m_PathFollowerPort("PathFollower")

	// </rtc-template>
{
}

/*!
* @brief destructor
*/
SimplePathFollower::~SimplePathFollower()
{
}



RTC::ReturnCode_t SimplePathFollower::onInitialize()
{
	// Registration: InPort/OutPort/Service
	// <rtc-template block="registration">
	// Set InPort buffers
        //addInPort("path", m_pathIn);
	addInPort("currentPose", m_currentPoseIn);
	// Set OutPort buffer
	addOutPort("velocity", m_velocityOut);

	// Set service provider to Ports
	m_PathFollowerPort.registerProvider("PathFollower", "RTC::PathFollower", m_pathFollower);

	// Set service consumers to Ports

	// Set CORBA Service Ports
	addPort(m_PathFollowerPort);
	m_pathFollower.setRTC(this);

	// </rtc-template>

	// <rtc-template block="bind_config">
	// Bind variables and configuration variable
	bindParameter("debug", m_debug, "0");
	bindParameter("directionToTranslationGain", m_directionToTranslationGain, "1.0");
	bindParameter("distanceToTranslationGain", m_distanceToTranslationGain, "1.0");
	bindParameter("directionToRotationGain", m_directionToRotationGain, "1.0");
	bindParameter("distanceToRotationGain", m_distanceToRotationGain, "1.0");
	bindParameter("maxVelocity", m_maxVelocity, "1.0");
	bindParameter("minVelocity", m_minVelocity, "1.0");
	bindParameter("approachDirectionGain", m_approachDirectionGain, "1.0");
	bindParameter("approachDistanceGain", m_approachDistanceGain, "1.0");
	bindParameter("poseTimeout", m_poseTimeout, "3.0");
	// </rtc-template>

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimplePathFollower::onFinalize()
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onStartup(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onShutdown(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SimplePathFollower::onActivated(RTC::UniqueId ec_id)
{
	m_poseUpdated = false;
	m_Mode = MODE_NORMAL;
	m_pathFollowerObj.stopFollow();
	m_lastReceivedTime = coil::gettimeofday();
	return RTC::RTC_OK;
}


RTC::ReturnCode_t SimplePathFollower::onDeactivated(RTC::UniqueId ec_id)
{
	return RTC::RTC_OK;
}


RTC::ReturnCode_t SimplePathFollower::onExecute(RTC::UniqueId ec_id)
{

	coil::TimeValue currentTime = coil::gettimeofday();
	if (m_currentPoseIn.isNew()) {
		m_currentPoseIn.read();
		m_poseUpdated = true;

		m_lastReceivedTime = currentTime;
	} else {
		double duration = currentTime - m_lastReceivedTime;
		if (duration > m_poseTimeout && m_poseTimeout > 0) {
			m_Mode = MODE_TIMEOUT;
		}
	}

	//if (m_debug) {

	if(m_pathFollowerObj.isFollowing()) {


		if (m_poseUpdated) {
			m_pathFollowerObj.setGain(m_maxVelocity, m_minVelocity,
				m_distanceToTranslationGain, m_directionToTranslationGain,
				m_distanceToRotationGain, m_directionToRotationGain,
				m_approachDistanceGain, m_approachDirectionGain);

			if (false) {
				static int i = 0;
				if(i++ % 30 == 0) {
					std::cout << "[RTC::SimplePathFollower] Now Following." << std::endl;
					std::cout << "[RTC::SimplePathFollower] Current Pose (" << m_currentPose.data.position.x << ", "
						<< m_currentPose.data.position.y << ", "
						<< m_currentPose.data.heading << ")" << std::endl;

				}
			}


			m_pathFollowerObj.setCurrentPose(m_currentPose.data);
			FOLLOW_RESULT ret = m_pathFollowerObj.follow();

			if (ret == FOLLOW_DISTANCEOUTOFRANGE) {
				m_Mode = MODE_OUTOFRANGE;
				m_pathFollowerObj.stopFollow();
			}

			if(m_pathFollowerObj.isGoal()) 
			{
				m_pathFollowerObj.stopFollow();
				m_Mode = MODE_GOALED;
			}


			m_pathFollowerObj.getTargetVelocity(m_velocity.data);
			
			if (false) {
				static int j = 0;
				if(++j % 30 == 0) {
					std::cout << "[RTC::SimplePathFollower] Target Velocity (" << m_velocity.data.vx << ", "
						<< m_velocity.data.vy << ", "
						<< m_velocity.data.va << ")" << std::endl;

				}
			}
			setTimestamp(m_velocity);
			m_velocityOut.write();
			m_poseUpdated = false;
		}
	}

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimplePathFollower::onAborting(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onError(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onReset(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onStateUpdate(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onRateChanged(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/



extern "C"
{

	void SimplePathFollowerInit(RTC::Manager* manager)
	{
		coil::Properties profile(simplepathfollower_spec);
		manager->registerFactory(profile,
			RTC::Create<SimplePathFollower>,
			RTC::Delete<SimplePathFollower>);
	}

};


