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
#include <sstream>
#include <fstream>
#include <sdf/SDFImpl.hh>


#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_glider_plugin/liftdrag_glider_plugin.h"
#include "aerodynamics/aero_forces_moments.h"
#include "aerodynamics/aero_lookup.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragGliderPlugin)

/////////////////////////////////////////////////
LiftDragGliderPlugin::LiftDragGliderPlugin()
{
  // Aircraft geometry
  _area = 0.0;
  _span = 0.0;
  _chord = 0.0;

  // Initialize aero variables
  _rho = 1.225f;

  this->wind_vel_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
}

/////////////////////////////////////////////////
LiftDragGliderPlugin::~LiftDragGliderPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragGliderPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragGliderPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragGliderPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragGliderPlugin world pointer is NULL");

  this->physics = this->world->Physics();
  this->last_pub_time = this->world->SimTime();

  GZ_ASSERT(this->physics, "LiftDragGliderPlugin physics pointer is NULL");

  // Read aircraft properties from .sdf
  getSdfParam<float>(_sdf, "area", _area, _area);
  getSdfParam<float>(_sdf, "span", _span, _span);
  getSdfParam<float>(_sdf, "chord", _chord, _chord);

  GZ_ASSERT(_sdf, "LiftDragGliderPlugin _sdf pointer is NULL");


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
        << "The LiftDragGliderPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragGliderPlugin::OnUpdate, this));
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

  if (_sdf->HasElement("windSubTopic")){
    this->wind_sub_topic_ = _sdf->Get<std::string>("windSubTopic");
    wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &LiftDragGliderPlugin::WindVelocityCallback, this);
  }

  // Read Aircraft Aerodynamics
  std::string aero_alpha_filepath;
  std::string aero_mach_filepath;
  std::string aero_xc_filepath;
  std::string aero_coeff_filepath;
  std::string aero_sym_filepath;
  std::string aero_asym_filepath;

  bool missing_param = false;
  missing_param |= !getSdfParam<std::string>(_sdf, "AeroAlpha_FilePath", aero_alpha_filepath,
                          aero_alpha_filepath);
  missing_param |= !getSdfParam<std::string>(_sdf, "AeroMach_FilePath", aero_mach_filepath,
                          aero_mach_filepath);
  missing_param |= !getSdfParam<std::string>(_sdf, "AeroXc_FilePath", aero_xc_filepath,
                          aero_xc_filepath);
  missing_param |= !getSdfParam<std::string>(_sdf, "AeroCoeff_FilePath", aero_coeff_filepath,
                          aero_coeff_filepath);
  missing_param |= !getSdfParam<std::string>(_sdf, "AeroSym_FilePath", aero_sym_filepath,
                          aero_sym_filepath);
  missing_param |= !getSdfParam<std::string>(_sdf, "AeroAsym_FilePath", aero_asym_filepath,
                          aero_asym_filepath);

  if (missing_param) {
    gzerr << "[liftdrag_glider_plugin] Missing Aero table filepath parameter.\n";
  }

  out_alpha = read_csv(sdf::findFile(aero_alpha_filepath));
  out_mach = read_csv(sdf::findFile(aero_mach_filepath));
  out_xc = read_csv(sdf::findFile(aero_xc_filepath));
  out = read_csv(sdf::findFile(aero_coeff_filepath));
  out_sym = read_csv(sdf::findFile(aero_sym_filepath));
  out_asym = read_csv(sdf::findFile(aero_asym_filepath));

  if (out_alpha.empty() || out_mach.empty() || out_xc.empty() || out.empty() || out_sym.empty() || out_asym.empty() ) {
    gzerr << "[liftdrag_glider_plugin] Aero tables incorrectly loaded.\n";
  }

  for (int i = 0; i < out_alpha.size(); i++){
    if (out_alpha.at(i).first == "Alpha"){
      ALPHA = out_alpha.at(i).second;
    }
  }

  for (int i = 0; i < out_mach.size(); i++){
    if (out_mach.at(i).first == "Mach"){
      MACH = out_mach.at(i).second;
    }
  }

  for (int i = 0; i < out_xc.size(); i++){
    if (out_xc.at(i).first == "XC"){
      XC = out_xc.at(i).second;
    }
  }

  for (int i = 0; i < out.size(); i++){
    if (out.at(i).first == "CD"){
      cd = out.at(i).second;
    }
    if (out.at(i).first == "CL"){
      cl = out.at(i).second;
    }
    if (out.at(i).first == "Cm"){
      cm = out.at(i).second;
    }
    if (out.at(i).first == "CYb"){
      cyb = out.at(i).second;
    }
    if (out.at(i).first == "Cnb"){
      cnb = out.at(i).second;
    }
    if (out.at(i).first == "Clb"){
      clb = out.at(i).second;
    }
    if (out.at(i).first == "CLq"){
      clq = out.at(i).second;
    }
    if (out.at(i).first == "Cmq"){
      cmq = out.at(i).second;
    }
    if (out.at(i).first == "CLad"){
      clad = out.at(i).second;
    }
    if (out.at(i).first == "Cmad"){
      cmad = out.at(i).second;
    }
    if (out.at(i).first == "Clp"){
      clp = out.at(i).second;
    }
    if (out.at(i).first == "CYp"){
      cyp = out.at(i).second;
    }
    if (out.at(i).first == "Cnp"){
      cnp = out.at(i).second;
    }
    if (out.at(i).first == "Cnr"){
      cnr = out.at(i).second;
    }
    if (out.at(i).first == "Clr"){
      clr = out.at(i).second;
    }
  }

  for (int i = 0; i < out_sym.size(); i++){
    if (out_sym.at(i).first == "DCL"){
      dcl = out_sym.at(i).second;
    }
    if (out_sym.at(i).first == "DCm"){
      dcm = out_sym.at(i).second;
    }
    if (out_sym.at(i).first == "DCl"){
      clroll = out_sym.at(i).second;
    }
  }

  for (int i = 0; i < out_asym.size(); i++){
    if (out_asym.at(i).first == "DCD"){
      dcdi = out_asym.at(i).second;
    }
    if (out_asym.at(i).first == "DCn"){
      cn_asy = out_asym.at(i).second;
    }
  }

  len_a = ALPHA.size();
  len_m = MACH.size();
  len_xc = XC.size();

}

/////////////////////////////////////////////////
void LiftDragGliderPlugin::OnUpdate()
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
  ignition::math::Vector3d Vel_wind_b = q_ENU_to_FRD.RotateVector(wind_vel_);
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
  double Delta[3] = {controlAngle_lail * 180.0 / M_PI,
                    controlAngle_elev * 180.0 / M_PI,
                    controlAngle_rudd * 180.0 / M_PI};

  alpha = atan2(V_r.Z(), V_r.X()) * 180.0 / M_PI;
  u = {(float) Delta[0], (float) Delta[1]};

  aero_lookup(alpha, u, ALPHA, XC, cd, cl, cm, cyb, cnb,
              clb, clq, cmq, clad, cmad, clp, cyp, cnp, cnr, clr,
              dcl, dcm, dcdi, clroll, cn_asy, &CD, &CL, &Cm, &CYb,
              &Cnb, &Clb, &CLq, &Cmq, &CLad, &Cmad, &Clp, &CYp, &Cnp,
              &Cnr, &Clr, &DCL, &DCm, &DCD, &DCl, &DCn, len_a, len_xc);

  // // Initialize Forces and Moments variables
  float Fx;
  float Fy;
  float Fz;
  float Mx;
  float My;
  float Mz;
	float L;
	float D;
  float uu[3] = {(float) Delta[0], (float) Delta[1], (float) Delta[2]};

  // Get aerodynamic forces and moments
  aero_forces_moments(V_r.X(), V_r.Y(), V_r.Z(), omega_rel_b.X(),
                      omega_rel_b.Y(), omega_rel_b.Z(), _rho,
                      _chord, _span, _area, uu,
                      CD, CL, Cm, CYb, Cnb, Clb, CLq, Cmq, CLad, Cmad, Clp, CYp,
                      Cnp, Cnr, Clr, DCL, DCm, DCD, DCl, DCn,
                      &Fx, &Fy, &Fz, &Mx, &My, &Mz, &L, &D);

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

  if (0)
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

void LiftDragGliderPlugin::WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}

std::vector<std::pair<std::string, std::vector<float>>> LiftDragGliderPlugin::read_csv(std::string filename){
  // Reads a CSV file into a vector of <string, vector<int>> pairs where
  // each pair represents <column name, column values>

  // Create a vector of <string, int vector> pairs to store the result
  std::vector<std::pair<std::string, std::vector<float>>> result;

  // Create an input filestream
  std::ifstream myFile(filename);

  // Helper vars
  std::string line, colname;
  //    std::string token;
  float val;

  // Read the column names
  if(myFile.good())
  {
    // Extract the first line in the file
    std::getline(myFile, line);

    // Create a stringstream from line
    std::stringstream ss(line);

    // Extract each column name
    while(std::getline(ss, colname, ',')){
      // Initialize and add <colname, int vector> pairs to result
      result.push_back({colname, std::vector<float> {}});
      //std::cout << colname << std::endl;
    }
    // std::cout << result.size() << std::endl;

  }

  // Read data, line by line
  while(std::getline(myFile, line))
  {
    // Create a stringstream of the current line
    std::stringstream ss(line);
    //std::cout << ss.str() << std::endl;

    // Keep track of the current column index
    int colIdx = 0;

    // Extract each value (OLD: while (ss >> val))
    //while(std::getline(ss, token, ',')){
    while(ss >> val){
      // Read current stringstream token and convert to float
      //val = std::stof(token);

      // Add the current value to the 'colIdx' column's values vector
      result.at(colIdx).second.push_back(val);

      // If the next token is a comma, ignore it and move on
      if(ss.peek() == ',') ss.ignore();

      // Increment the column index
      colIdx++;

      // std::cout << colIdx << ": " << val << std::endl;
    }
  }
  // Close file
  myFile.close();

  return result;
}
