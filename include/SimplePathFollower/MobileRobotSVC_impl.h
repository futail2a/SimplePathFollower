// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.h
 * @brief Service implementation header of MobileRobot.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSkel.h"

#ifndef MOBILEROBOTSVC_IMPL_H
#define MOBILEROBOTSVC_IMPL_H
 

class SimplePathFollower;
/*!
 * @class PathFollowerSVC_impl
 * Example class implementing IDL interface RTC::PathFollower
 */
class PathFollowerSVC_impl
 : public virtual POA_RTC::PathFollower,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~PathFollowerSVC_impl();
   SimplePathFollower* m_pRTC;

 public:
  /*!
   * @brief standard constructor
   */
   PathFollowerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~PathFollowerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE followPath(const RTC::Path2D& path);

   RTC::RETURN_VALUE getState(::RTC::FOLLOWER_STATE& state);

   RTC::RETURN_VALUE followPathNonBlock(const ::RTC::Path2D& path);

   void setRTC(SimplePathFollower* pRTC) {
	 m_pRTC = pRTC;
   }

   virtual RTC::Path2D refleshArgs();

};

class PortDecorator : public PathFollowerSVC_impl{
 private:
   PathFollowerSVC_impl* pPathFollowerSVC_impl;
   ::RTC::Path2D m_path;
   ::RTC::FOLLOWER_STATE m_state;

public :
	PortDecorator();
	~PortDecorator();

	 	
   RTC::RETURN_VALUE followPath(const RTC::Path2D& path);

   RTC::RETURN_VALUE getState(::RTC::FOLLOWER_STATE& state);

   RTC::RETURN_VALUE followPathNonBlock(const ::RTC::Path2D& path);

   void setRTC(SimplePathFollower* pRTC);
 
};


#endif // MOBILEROBOTSVC_IMPL_H


