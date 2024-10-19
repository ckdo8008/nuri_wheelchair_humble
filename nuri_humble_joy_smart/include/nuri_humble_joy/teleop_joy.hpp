#ifndef TELEOP_JOY_H
#define TELEOP_JOY_H

#include <rclcpp/rclcpp.hpp>
#include "teleop_twist_joy/teleop_twist_joy_export.h"

class TELEOP_TWIST_JOY_EXPORT TeleopJoy : public rclcpp::Node
{
public:
  explicit TeleopJoy(const rclcpp::NodeOptions &options);
  virtual ~TeleopJoy();

private:
  struct Impl;
  Impl *pimpl_;
  // OnSetParametersCallbackHandle::SharedPtr callback_handle;
};

#endif