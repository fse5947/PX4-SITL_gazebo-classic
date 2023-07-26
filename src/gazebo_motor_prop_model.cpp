/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "gazebo_motor_prop_model.h"
#include <ignition/math.hh>

namespace gazebo {

GazeboMotorPropModel::~GazeboMotorPropModel() {
  updateConnection_->~Connection();

}

void GazeboMotorPropModel::InitializeParams() {}

void GazeboMotorPropModel::Publish() {
  //turning_velocity_msg_.set_data(joint_->GetVelocity(0));
  // FIXME: Commented out to prevent warnings about queue limit reached.
  // motor_velocity_pub_->Publish(turning_velocity_msg_);
  power_msg_.set_data(power_);
  motor_power_pub_->Publish(power_msg_);
}

void GazeboMotorPropModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_prop_model] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_prop_model] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_prop_model] Couldn't find specified joint \"" << joint_name_ << "\".");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_prop_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_prop_model] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("maxJ")) {
    max_j_ = _sdf->GetElement("maxJ")->Get<double>();
  } else {
    gzthrow("[gazebo_motor_prop_model] Missing maxJ.");
  }

  if (_sdf->HasElement("diameter")) {
    propeller_diameter_ = _sdf->GetElement("diameter")->Get<double>();
  } else {
    gzthrow("[gazebo_motor_prop_model] Missing diameter.");
  }

  // char buffer [20];
  // for (int i=0; i<5; i++) {
  //   sprintf (buffer, "thrustPolyCoef%d", i);
  //   if (_sdf->HasElement(buffer)) {
  //     thrust_coefficients_[0] = _sdf->GetElement((std::string)buffer)->Get<double>();
  //   } else {
  //     gzthrow("[gazebo_motor_prop_model] Missing"+(std::string)buffer);
  //   }
  // }

  // if (_sdf->HasElement("powerPoly")) {
  //   power_coefficients_ = _sdf->GetElement("powerPoly")->Get<ignition::math::Vector4d>();
  // } else {
  //   gzthrow("[gazebo_motor_prop_model] Missing powerPoly.");
  // }

  // if (thrust_coefficients_ != 4 || power_coefficients_!=4){
  //   gzthrow("[gazebo_motor_prop_model] Incorrect size of coefficients vectors.");
  // }


  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_prop_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_prop_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_prop_model] Please specify a turning direction ('cw' or 'ccw').\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "motorPowerPubTopic", motor_power_pub_topic_,
                           motor_power_pub_topic_);
  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                      rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "efficiency", efficiency_, efficiency_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorPropModel::OnUpdate, this, _1));

  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &GazeboMotorPropModel::VelocityCallback, this);
  //std::cout << "[gazebo_motor_prop_model]: Subscribe to gz topic: "<< motor_failure_sub_topic_ << std::endl;
  motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &GazeboMotorPropModel::MotorFailureCallback, this);
  // FIXME: Commented out to prevent warnings about queue limit reached.
  //motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);
  motor_power_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_power_pub_topic_, 1);
  wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &GazeboMotorPropModel::WindVelocityCallback, this);

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// Protobuf test
/*
void GazeboMotorPropModel::testProto(MotorSpeedPtr &msg) {
  std::cout << "Received message" << std::endl;
  std::cout << msg->motor_speed_size()<< std::endl;
  for(int i; i < msg->motor_speed_size(); i++){
    std::cout << msg->motor_speed(i) <<" ";
  }
  std::cout << std::endl;
}
*/

// This gets called by the world update start event.
void GazeboMotorPropModel::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  // Apply the filter on the motor's velocity.
  const double ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
  UpdateMotorFail();
  Publish();
}

void GazeboMotorPropModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else {
    ref_motor_rot_vel_ = static_cast<double>(rot_velocities->motor_speed(motor_number_));
  }
}

void GazeboMotorPropModel::MotorFailureCallback(IntPtr &fail_msg) {
  motor_Failure_Number_ = fail_msg->data();
}

void GazeboMotorPropModel::UpdateForcesAndMoments() {
  motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;

  double motor_velocity_hz = std::abs(real_motor_velocity) / (2 * M_PI);

  if (motor_velocity_hz <= 0.1) {
    return;
  }

  // scale down force linearly with forward speed
  // XXX this has to be modelled better
  //
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link_->GetWorldLinearVel());
  ignition::math::Vector3d joint_axis = ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
#endif

  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;
  ignition::math::Vector3d velocity_parallel_to_rotor_axis = (relative_wind_velocity.Dot(joint_axis)) * joint_axis;

  double va = velocity_parallel_to_rotor_axis.Length();
  double J = va / (motor_velocity_hz * propeller_diameter_);

  J = std::min(max_j_, J);

  double CT = 0.0;
  double CP = 0.0;

  for (int i=0; i < PROPPOLYDEGREE; i++) {
    CT += thrust_coefficients_[i] * pow(J,i);
    CP += power_coefficients_[i] * pow(J,i);
  }

  double thrust = CT * rho * pow(motor_velocity_hz,2.0) * pow(propeller_diameter_,4.0);
  double propeller_power = CP * rho * pow(motor_velocity_hz,3.0) * pow(propeller_diameter_,5.0);

  double torque = propeller_power / std::abs(real_motor_velocity);

  power_= propeller_power / efficiency_;

  // gzerr << "Motor hz :" << motor_velocity_hz << ", "
  //  << "Va error: " << va - relative_wind_velocity.Length() << ", "
  //  << "Va: " << va << ", "
  //  << "J: " << J << ", "
  //  << "CT: " << CT << ", "
  //  << "CP: " << CP << ", "
  //  << "T: " << thrust << ", "
  //  << "P: " << power_ << ", "
  //  << "Q: " << torque << '\n';

  // Apply a force to the link.
  link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

  // Getting the parent link, such that the resulting torques can be applied to it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The transformation from the parent_link to the link_.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
#else
  ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
#endif
  ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * torque);
  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);


#if 0
  //? Forces from Philppe Martin's and Erwan Salaün's
  //? 2010 IEEE Conference on Robotics and Automation paper
  //? The True Role of Accelerometer Feedback in Quadrotor Control
  //? - \omega * \lambda_1 * V_A^{\perp}
  ignition::math::Vector3d velocity_perpendicular_to_rotor_axis = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
  // Apply air_drag to link.
  link_->AddForce(air_drag);

  //* Moments

  ignition::math::Vector3d rolling_moment;
  //? - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * turning_direction_ * rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
  parent_links.at(0)->AddTorque(rolling_moment);
#endif

}

void GazeboMotorPropModel::UpdateMotorFail() {
  if (motor_number_ == motor_Failure_Number_ - 1){
    // motor_constant_ = 0.0;
    joint_->SetVelocity(0,0);
    if (screen_msg_flag){
      std::cout << "Motor number [" << motor_Failure_Number_ <<"] failed!  [Motor thrust = 0]" << std::endl;
      tmp_motor_num = motor_Failure_Number_;

      screen_msg_flag = 0;
    }
  }else if (motor_Failure_Number_ == 0 && motor_number_ ==  tmp_motor_num - 1){
     if (!screen_msg_flag){
       //motor_constant_ = kDefaultMotorConstant;
       std::cout << "Motor number [" << tmp_motor_num <<"] running! [Motor thrust = (default)]" << std::endl;
       screen_msg_flag = 1;
     }
  }
}

void GazeboMotorPropModel::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPropModel);
}
