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
Measurement<MsgT>::Measurement(
    const Msg& msg,
    Callback callback,
    const rclcpp::Time& stamp
) :
    msg_(msg),
    callback_(callback),
    stamp_(stamp)
{
}


template<typename MsgT>
rclcpp::Time Measurement<MsgT>::stamp() const {
    if (stamp_) {
        return *stamp_;
    }
    if constexpr (header<MsgT>::value) {
        return rclcpp::Time(msg_->header.stamp);
    } else {
        throw std::runtime_error("Message does not have header.stamp and no override was provided.");
    }
}



template<typename MsgT>
void Measurement<MsgT>::dispatch(
) {
    callback_(*msg_);
}

} // end namespace filters_base

#endif // _FILTERS_BASE_MEASUREMENT_IMPL_HPP
