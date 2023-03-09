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
 * @author Alexandre Borowczyk <alexandre.borowczyk@shearwater.ai>
 *
 */

#ifndef _GAZEBO_BATTERY_PLUGIN_HH_
#define _GAZEBO_BATTERY_PLUGIN_HH_

#include <common.h>
#include <sdf/sdf.hh>

#include <string>

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>

#include "Float.pb.h"
#include "Battery.pb.h"

namespace gazebo {

  static const std::string DEFAULT_BATTERY_TOPIC = "/battery";
  static const std::string DEFAULT_MOTOR_POWER_TOPIC = "/motor_power";

  static constexpr unsigned int DEFAULT_PUB_RATE = 50;  // [Hz].
  static constexpr double DEFAULT_CAPACITY = 100.0; // [mWh]
  static constexpr double DEFAULT_NOMINAL_VOLTAGE = 16.8; // [V]

  typedef const boost::shared_ptr<const std_msgs::msgs::Float> FloatPtr;

  class BatteryPlugin : public ModelPlugin {
  public:
    BatteryPlugin() : ModelPlugin(),
    power_(0.0),
    pub_rate_(DEFAULT_PUB_RATE),
    battery_topic_(DEFAULT_BATTERY_TOPIC),
    current_capacity_(DEFAULT_CAPACITY),
    nominal_capacity_(DEFAULT_CAPACITY),
    nominal_voltage_(DEFAULT_NOMINAL_VOLTAGE),
    motor_power_sub_topic_(DEFAULT_MOTOR_POWER_TOPIC)
    {};

    virtual ~BatteryPlugin();

  protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void OnUpdate(const common::UpdateInfo&);
    void getSdfParams(sdf::ElementPtr sdf);
    void PowerCallback(FloatPtr  &power_msg);

  private:
    std::string namespace_;
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;
    std::string battery_topic_;
    std::string motor_power_sub_topic_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr pub_battery_;
    transport::SubscriberPtr motor_power_sub_;

    sensor_msgs::msgs::Battery battery_msg_;
    unsigned int pub_rate_;


    common::Time last_pub_time_;
    common::Time last_time_;

    double power_;
    double nominal_voltage_;
    double current_capacity_;
    double nominal_capacity_;

  }; // class BatteryPlugin
} // namespace gazebo
#endif // _GAZEBO_BATTERY_PLUGIN_HH_
