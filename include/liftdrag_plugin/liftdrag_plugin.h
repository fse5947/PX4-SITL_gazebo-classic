/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_LIFT_DRAG_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_PLUGIN_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

#include "Wind.pb.h"

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE LiftDragPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LiftDragPlugin();

    /// \brief Destructor.
    public: ~LiftDragPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Air density
    protected: double _rho;
    /// \brief Planform surface area
    protected: double _area;
    /// \brief Wing span
    protected: double _span;
    /// \brief Wing chord length
    protected: double _chord;
    /// \brief Angle of Attack
    protected: double alpha;

		/// \brief model mass
    protected: double _mass;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to a joint that actuates the left aileron for
    /// this lifting body
    protected: physics::JointPtr controlJoint_left_ail;

    /// \brief Pointer to a joint that actuates the right aileron for
    /// this lifting body
    protected: physics::JointPtr controlJoint_right_ail;

    /// \brief Pointer to a joint that actuates the elevator for
    /// this lifting body
    protected: physics::JointPtr controlJoint_elev;

    /// \brief Pointer to a joint that actuates the rudder for
    /// this lifting body
    protected: physics::JointPtr controlJoint_rudd;
    // /// \brief how much to change CL per radian of control surface joint
    // /// value.
    // protected: double controlJointRadToCL;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    private: void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg);

    private: transport::NodePtr node_handle_;
    private: transport::SubscriberPtr wind_sub_;
    private: transport::PublisherPtr lift_force_pub_;
    private: common::Time last_pub_time;
    private: msgs::Factory msg_factory_;
    private: std::string namespace_;
    private: std::string wind_sub_topic_ = "world_wind";
    private: ignition::math::Vector3d wind_vel_;
  };
}
#endif
