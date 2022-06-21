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

#include <algorithm>
#include <string>
#include <iostream>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/liftdrag_plugin.h"
#include "aerodynamics/aero_forces_moments.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin()
{
  // Aircraft geometry
  _area = 0.0;
  _span = 0.0;
  _chord = 0.0;

  // Initialize aero variables
  _rho = 1.225f;
}

/////////////////////////////////////////////////
LiftDragPlugin::~LiftDragPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

  this->physics = this->world->Physics();
  this->last_pub_time = this->world->SimTime();

  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  // Read aircraft properties from .sdf
  getSdfParam<double>(_sdf, "area", _area, _area);
  getSdfParam<double>(_sdf, "span", _span, _span);
  getSdfParam<double>(_sdf, "chord", _chord, _chord);

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");


  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_liftdrag_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  std::string joint_name;
  getSdfParam<std::string>(_sdf, "control_joint_left_ail", joint_name, "");
  this->controlJoint_left_ail = this->model->GetJoint(joint_name);
  if (!this->controlJoint_left_ail)
  {
    gzerr << "Joint with name 'control_joint_left_ail' does not exist.\n";
  }
  getSdfParam<std::string>(_sdf, "control_joint_right_ail", joint_name, "");
  this->controlJoint_right_ail = this->model->GetJoint(joint_name);
  if (!this->controlJoint_right_ail)
  {
    gzerr << "Joint with name 'control_joint_right_ail' does not exist.\n";
  }
  getSdfParam<std::string>(_sdf, "control_joint_elev", joint_name, "");
  this->controlJoint_elev = this->model->GetJoint(joint_name);
  if (!this->controlJoint_elev)
  {
    gzerr << "Joint with name 'control_joint_elev' does not exist.\n";
  }
  getSdfParam<std::string>(_sdf, "control_joint_rudd", joint_name, "");
  this->controlJoint_rudd = this->model->GetJoint(joint_name);
  if (!this->controlJoint_rudd)
  {
    gzerr << "Joint with name 'control_joint_rudd' does not exist.\n";
  }

}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{

  // std::cout << "TESTING LIFTDRAG" << std::endl;

  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame

  ignition::math::Vector3d vel_ENU = this->link->WorldLinearVel();

  // ignition::math::Vector3d velI = vel;
  // velI.Normalize();
  // const double dt = (current_time - this->last_pub_time).Double();

  // pose of body
  ignition::math::Pose3d pose = this->link->WorldPose();

    // Attitude calculations
  ignition::math::Quaterniond q_FLU_to_ENU = pose.Rot(); // Quat defining rotation from FLU to ENU
  ignition::math::Quaterniond q_FRD_to_ENU = q_FLU_to_ENU * q_FLU_to_FRD.Inverse(); // From FRD to ENU
	ignition::math::Quaterniond q_ENU_to_FRD = q_FRD_to_ENU.Inverse(); // From FRD to ENU

  ignition::math::Quaterniond q_FRD_to_NED = q_ENU_to_NED * q_FRD_to_ENU; // From FRD to NED
	ignition::math::Quaterniond q_NED_to_FRD = q_FRD_to_NED.Inverse();
  //Vector3d Eul = q_FRD_to_NED.Euler(); // Euler angles in NED frame

  // Velocities
  // Body Velocity in Px4 body frame (FRD)
  ignition::math::Vector3d Vel_b = q_FLU_to_FRD.RotateVector(this->link->RelativeLinearVel()); // FLU to FRD
  ignition::math::Vector3d Vel_wind_b(0,0,0); // = q_ENU_to_FRD.RotateVector(_wind_speed);
  ignition::math::Vector3d V_r = Vel_b - Vel_wind_b; // Relative Velocity

	double TAS = V_r.Length();

  if (TAS <= 0.02) {
		return;
	}

  // // Angular Velocities

  // Body Angular Velocity in Px4 body frame (FRD)
  ignition::math::Vector3d omega_nb_b = q_FLU_to_FRD.RotateVector(this->link->RelativeAngularVel());
	// Wind induced Angular Velocity
  ignition::math::Vector3d Vel_wind_b_grad(0, 0, 0); // = q_ENU_to_FRD.RotateVector(_wind_gradient); // Rotate to body frame
  ignition::math::Vector3d omega_w_b = ignition::math::Vector3d(-1 * Vel_wind_b_grad.Y(), Vel_wind_b_grad.X(), 0); // From ICUAS paper
  // Wind relative Angular Velocity
  ignition::math::Vector3d omega_rel_b = omega_nb_b + omega_w_b;

  // Get control surface deflections
  double controlAngle_lail = -1 * this->controlJoint_left_ail->Position(0);
  double controlAngle_rail = -1 * this->controlJoint_right_ail->Position(0);
  double controlAngle_elev = -1 * this->controlJoint_elev->Position(0);
  double controlAngle_rudd = -1 * this->controlJoint_rudd->Position(0);
  double Delta[3] = {controlAngle_lail * 180 / M_PI,
                    controlAngle_elev * 180 / M_PI,
                    controlAngle_rudd * 180 / M_PI};

  // Initialize Forces and Moments variables
	double alpha_deg;
  double Fx;
  double Fy;
  double Fz;
  double Mx;
  double My;
  double Mz;
	double L;
	double D;

  // Get aerodynamic forces and moments
  aero_forces_moments(V_r.X(), V_r.Y(), V_r.Z(), omega_rel_b.X(),
                      omega_rel_b.Y(), omega_rel_b.Z(), Delta, _rho,
                      _chord, _span, _area, &alpha_deg, &Fx, &Fy, &Fz, &Mx, &My, &Mz, &L, &D);

  ignition::math::Vector3d Force_b = ignition::math::Vector3d(Fx, Fy, Fz);
  ignition::math::Vector3d Moment_b = ignition::math::Vector3d(Mx, My, Mz);

  // Resolve forces and moments in the gazebo inertial frame
  ignition::math::Vector3d Force_g = q_FRD_to_ENU * Force_b;
  ignition::math::Vector3d Moment_g = q_FRD_to_ENU * Moment_b;

  // Correct for nan or inf
  Force_g.Correct();
  // this->cp.Correct();
  Moment_g.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForce(Force_g);
  this->link->AddTorque(Moment_g);

  if (1)
  {
    gzdbg << "=============================\n";
    // gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    // gzdbg << "Link: [" << this->link->GetName()
    gzdbg << "Aileron deflection: " << controlAngle_lail * 180 / M_PI << "\n";
    gzdbg << "Elevator deflection: " << controlAngle_elev * 180 / M_PI << "\n";
    gzdbg << "Rudder deflection: " << controlAngle_rudd * 180 / M_PI << "\n";
    std::cout << "Vb: [" << V_r << "]\n";
    // gzdbg << "Vel_wind_b: [" << Vel_wind_b << "]\n";
    // gzdbg << "V_w_grad: [" << V_w_grad << "]\n";
    // gzdbg << "omega_w_b: [" << omega_w_b << "]\n";
    // gzdbg << "alpha: " << alpha * 180 / M_PI << "\n";
    std::cout << "Force_b: " << Force_b << std::endl;
    std::cout << "Moment_b: " << Moment_b << "\n";
    // gzdbg << "Position: [" << position << "]\n";
    // gzdbg << "V_w_grad: [" << _wind_gradient << "]\n";
    // gzdbg << "Vel_wind_b_grad: [" << Vel_wind_b_grad << "]\n";
  }
}
