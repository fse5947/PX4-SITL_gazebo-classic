/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Battery Plugin
 *
 * This plugin simulates battery data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 *
 * References in header.
 */

#include <gazebo_battery_plugin.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)

BatteryPlugin::~BatteryPlugin()
{
  update_connection_->~Connection();
}

void BatteryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_battery_plugin] Please specify a robotNamespace.\n";
  }

  getSdfParam<std::string>(sdf,"batteryTopic",battery_topic_,battery_topic_,true);
  getSdfParam<std::string>(sdf,"motorPowerSubTopic",motor_power_sub_topic_,motor_power_sub_topic_,true);
  getSdfParam<unsigned int>(sdf,"pubRate",pub_rate_,pub_rate_,true);
  getSdfParam<double>(sdf,"nominalCapacity",nominal_capacity_,nominal_capacity_,true);
  getSdfParam<double>(sdf,"capacity",current_capacity_,nominal_capacity_,true);
  current_capacity_ = std::min(current_capacity_,nominal_capacity_);
  getSdfParam<double>(sdf,"nominalVoltage",nominal_voltage_,nominal_voltage_,true);
  getSdfParam<double>(sdf,"efficiency",efficiency_,efficiency_,true);
}

void BatteryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  model_ = model;
  world_ = model_->GetWorld();

  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
  last_pub_time_ = world_->GetSimTime();
#endif

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BatteryPlugin::OnUpdate, this, _1));

  pub_battery_ = node_handle_->Advertise<sensor_msgs::msgs::Battery>("~/" + model_->GetName() + battery_topic_, 1);
  motor_power_sub_ = node_handle_->Subscribe<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_power_sub_topic_, &BatteryPlugin::PowerCallback, this);

}

void BatteryPlugin::PowerCallback(FloatPtr  &power_msg)
{
#if GAZEBO_MAJOR_VERSION >= 9
  const common::Time current_time = world_->SimTime();
#else
  const common::Time current_time = world_->GetSimTime();
#endif
  const double dt = (current_time - last_time_).Double();

  power_ = power_msg->data() / efficiency_;

  current_capacity_ -= power_ * dt / 3600.0;

  last_time_ = current_time;
}

void BatteryPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  const common::Time current_time = world_->SimTime();
#else
  const common::Time current_time = world_->GetSimTime();
#endif
  const double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_) {


    battery_msg_.set_voltage(nominal_voltage_);
    battery_msg_.set_current(power_ / nominal_voltage_);
    battery_msg_.set_remaining_pct(int32_t(current_capacity_ / nominal_capacity_ * 100));
    battery_msg_.set_remaining_capacity(current_capacity_);
    battery_msg_.set_nominal_voltage(nominal_voltage_);
    battery_msg_.set_nominal_capacity(nominal_capacity_);
    battery_msg_.set_temperature(25.0);

    last_pub_time_ = current_time;
    // Publish battery msg
    pub_battery_->Publish(battery_msg_);
  }
}
}
