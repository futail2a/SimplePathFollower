// -*-C++-*-

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSVC_impl.h"

class PortDecorator : public PathFollowerSVC_impl{
 private:
   PathFollowerSVC_impl* pPathFollowerSVC_impl;
   ::RTC::Path2D m_path;
   ::RTC::FOLLOWER_STATE m_state;

public :
	PortDecorator();
	~PortDecorator();

   virtual void refleshArgs();
	 	
   RTC::RETURN_VALUE followPath(const RTC::Path2D& path);

   RTC::RETURN_VALUE getState(::RTC::FOLLOWER_STATE& state);

   RTC::RETURN_VALUE followPathNonBlock(const ::RTC::Path2D& path);

   void setRTC(SimplePathFollower* pRTC) {
      pPathFollowerSVC_impl->setRTC(pRTC);
   }
};


#endif // MOBILEROBOTSVC_IMPL_H


