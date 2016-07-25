// -*-C++-*-

#include "PortDecorator.h"

PortDecorator::PortDecorator(){
	pPathFollowerSVC_impl = new PathFollowerSVC_impl();
}
PortDecorator::~PortDecorator(){}

void PortDecorator::refleshArgs(){
	//not implemented yet
	std::cout << "test decorator" << std::endl;
}

RTC::RETURN_VALUE PortDecorator::followPath(const RTC::Path2D& path)
{
	m_path = path;
	pPathFollowerSVC_impl->followPath(m_path);
	//while(pPathFollowerSVC_impl->followPath(m_path) != RTC::RETVAL_OK){
		refleshArgs();
	//}
	return RTC::RETVAL_OK;
}

RTC::RETURN_VALUE PortDecorator::getState(::RTC::FOLLOWER_STATE& state)
{
	m_state = state;
	return pPathFollowerSVC_impl->getState(m_state);
}

RTC::RETURN_VALUE PortDecorator::followPathNonBlock(const ::RTC::Path2D& path)
{
	m_path = path;
	return pPathFollowerSVC_impl->followPathNonBlock(m_path);
}

// End of example implementational code



