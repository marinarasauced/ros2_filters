#ifndef _FILTERS_BASE_MEASUREMENT_HPP
#define _FILTERS_BASE_MEASUREMENT_HPP

#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

namespace filters_base
{

class MeasurementInterface
{
public:
    virtual ~MeasurementInterface() = default;

    virtual rclcpp::Time stamp() const = 0;
    virtual void dispatch() = 0;

    bool operator<(const MeasurementInterface& other) const { return stamp().seconds() < other.stamp().seconds(); }
};


template<typename MsgT>
class Measurement : public MeasurementInterface
{
public:
    using Msg = std::shared_ptr<MsgT>;
    using Callback = std::function<void(const MsgT&)>;

    Measurement(const Msg& msg, Callback callback);

    rclcpp::Time stamp() const override;
    void dispatch() override;

private:
    Msg msg_;
    Callback callback_;
};

} // end namespace filters_base

#include "filters_base/sensors_impl.hpp"

#endif // _FILTERS_BASE_MEASUREMENT_HPP
