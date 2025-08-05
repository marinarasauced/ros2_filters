#ifndef _FILTERS_BASE_MEASUREMENT_IMPL_HPP
#define _FILTERS_BASE_MEASUREMENT_IMPL_HPP

#include "filters_base/sensors.hpp"

namespace filters_base
{

template<typename MsgT>
Measurement<MsgT>::Measurement(
    const Msg& msg,
    Callback callback
) :
    msg_(msg),
    callback_(callback)
{
}


template<typename MsgT>
rclcpp::Time Measurement<MsgT>::stamp(
) const {
    return rclcpp::Time(msg_->header.stamp);
}


template<typename MsgT>
void Measurement<MsgT>::dispatch(
) {
    callback_(msg_);
}

} // end namespace filters_base

#endif // _FILTERS_BASE_MEASUREMENT_IMPL_HPP
