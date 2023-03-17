/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
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

#include "gazebo_wind_plugin.h"
#include "common.h"

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  update_connection_->~Connection();
}

void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
  world_ = world;

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);


  getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  double pub_rate = 2.0;
  getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); //Wind topic publishing rates
  pub_interval_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;
  getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
  // Get the wind params from SDF.
  getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
  getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
  getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
  getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
  getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

  groundtruth_sub_ = node_handle_->Subscribe("~/glider" + groundtruth_sub_topic_, &GazeboWindPlugin::GroundtruthCallback, this);

  wind_direction_mean_.Normalize();
  wind_gust_direction_mean_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  // Set random wind velocity mean and standard deviation
  wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
  // Set random wind direction mean and standard deviation
  wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
  // Set random wind gust velocity mean and standard deviation
  wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
  // Set random wind gust direction mean and standard deviation
  wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));


  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  const bool world_has_origin = checkWorldHomePosition(world_, world_latitude_, world_longitude_, world_altitude_);

  if (env_lat) {
    lat_home_ = std::stod(env_lat) * M_PI / 180.0;
    gzmsg << "[gazebo_wind_plugin] Home latitude is set to " << std::stod(env_lat) << " (" << lat_home_ << ").\n";
  } else if (world_has_origin) {
    lat_home_ = world_latitude_;
    gzmsg << "[gazebo_wind_plugin] Home latitude is set to " << lat_home_ << ".\n";
  } else if(sdf->HasElement("homeLatitude")) {
    double latitude;
    getSdfParam<double>(sdf, "homeLatitude", latitude, lat_home_);
    lat_home_ = latitude * M_PI / 180.0;
  }

  if (env_lon) {
    lon_home_ = std::stod(env_lon) * M_PI / 180.0;
    gzmsg << "[gazebo_wind_plugin] Home longitude is set to " << std::stod(env_lon) << " (" << lon_home_ << ").\n";
  } else if (world_has_origin) {
    lon_home_ = world_longitude_;
    gzmsg << "[gazebo_wind_plugin] Home longitude is set to " << lon_home_ << ".\n";
  } else if(sdf->HasElement("homeLongitude")) {
    double longitude;
    getSdfParam<double>(sdf, "homeLongitude", longitude, lon_home_);
    lon_home_ = longitude * M_PI / 180.0;
  }

  if (env_alt) {
    alt_home_ = std::stod(env_alt);
    gzmsg << "[gazebo_wind_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if (world_has_origin) {
    alt_home_ = world_altitude_;
    gzmsg << "[gazebo_wind_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if(sdf->HasElement("homeAltitude")) {
    getSdfParam<double>(sdf, "homeAltitude", alt_home_, alt_home_);
  }

  //* Get thermal updraft params from SDF
  if (sdf->HasElement("Thermals")) {
		sdf::ElementPtr thermals = sdf->GetElement("Thermals");
		sdf::ElementPtr thermal = thermals->GetFirstElement();

		ignition::math::Vector3d centerCoordinates_DEG = ignition::math::Vector3d(0.0, 0.0, 0.0);

		while( thermal != nullptr && thermal->HasElement("centerCoordinates")) {

      ignition::math::Vector3d centerCoordinates_DEG;
      if (thermal->HasElement("centerCoordinates")) {
        centerCoordinates_DEG = thermal->GetElement("centerCoordinates")->Get<ignition::math::Vector3d>();
      } else {
        gzthrow("[gazebo_wind_plugin] Invalid thermal missing: centerCoordinates.\n" );
      }

      double thermal_strength;
      if (thermal->HasElement("ThermalStrength")) {
        thermal_strength = thermal->GetElement("ThermalStrength")->Get<double>();
      } else {
        gzthrow("[gazebo_wind_plugin] Invalid thermal missing: ThermalStrength.\n" );
      }

      double thermal_radius;
      if (thermal->HasElement("ThermalRadius")) {
        thermal_radius = thermal->GetElement("ThermalRadius")->Get<double>();
      } else {
        gzthrow("[gazebo_wind_plugin] Invalid thermal missing: ThermalRadius.\n" );
      }


      double alt{0.0};
      auto lat_rad = centerCoordinates_DEG.X()  / 180.0 * M_PI;
      auto lon_rad = centerCoordinates_DEG.Y() / 180.0 * M_PI;
			ignition::math::Vector3d centerCoordinates_NED = project(lat_rad, lon_rad, alt, lat_home_, lon_home_);

      thermal_manager_.addThermal(centerCoordinates_NED,thermal_strength,thermal_radius);

			gzdbg << "Adding thermal at lat: "<< centerCoordinates_DEG.X()<<", lon: "<< centerCoordinates_DEG.Y() << "], NED: ["<< centerCoordinates_NED.X()<<","<< centerCoordinates_NED.Y() << "]\n";

			thermal = thermal->GetNextElement();
		}

	}

  //* Get Thermal from file
  if (sdf->HasElement("thermalFile")) {
    std::string thermal_file;

    thermal_file =  sdf::findFile(sdf->GetElement("thermalFile")->Get<std::string>());
    gzmsg << "Reading Thermal from: " << thermal_file << '\n';

    std::ifstream file(thermal_file);
    Json::Reader reader;
    Json::Value thermal_data;
    reader.parse(file, thermal_data);
    gzmsg << "File has " << thermal_data["nbr_thermals"].asInt() << " thermals" << '\n';

    bool keep_ned{true};
    if (sdf->HasElement("keepThermalNED")) {
     keep_ned = sdf->GetElement("keepThermalNED")->Get<bool>();
    }

    double ref_lat{0.0}, ref_lon{0.0}, ref_alt{0.0};
    if (!keep_ned) {
      if (thermal_data.isMember("ref_lla")) {
        auto ref_lla = thermal_data["ref_lla"];
        ref_lat = ref_lla["lat"].asDouble();
        ref_lon = ref_lla["lon"].asDouble();
        ref_alt = ref_lla["alt"].asDouble();
        gzdbg << "Thermal ref Lat Lon [" << ref_lat << ", " << ref_lon << ", " << ref_alt << "]\n";
      } else {
        gzerr << "Missing reference coordinate for reprojection of thermals.\n";
      }
    } else {
      gzwarn << "Using thermal NED coordination base on home position\n";
    }

    if (thermal_data.isMember("env_sink")) {
      auto env_sink = thermal_data["env_sink"].asDouble();
      thermal_manager_.setEnvSink(env_sink);
      gzdbg << "Env sink: " << env_sink << '\n';
    }

    if (!thermal_data.isMember("thermals"))
      gzthrow("No thermals found in file.");

    auto thermals = thermal_data["thermals"];



    for (int i = 0; i < thermals.size(); i++){
      auto thermal = thermals[i];
      if (!thermal.isMember("n") || !thermal.isMember("e") || !thermal.isMember("d"))
        gzthrow("Thermal missing coordinate");
      if (!thermal.isMember("spawn_time"))
        gzthrow("Thermal missing spawn time");
      if (!thermal.isMember("eta"))
        gzthrow("Thermal missing rise time factor");
      if (!thermal.isMember("active_period"))
        gzthrow("Thermal missing active_period");
      if (!thermal.isMember("radius"))
        gzthrow("Thermal missing radius");
      if (!thermal.isMember("max_strength"))
        gzthrow("Thermal missing max strength");
      auto centerCoordinates_NED = ignition::math::Vector3d(thermal["n"].asDouble(), thermal["e"].asDouble(), thermal["d"].asDouble());

      if (!keep_ned) {
        //gzdbg << "NED: ["<< centerCoordinates_NED.X()<<","<< centerCoordinates_NED.Y() << "]\n";
        auto centerCoordinates_ENU = q_ENU_to_NED.Inverse().RotateVector(centerCoordinates_NED);
        //gzdbg << "ENU: ["<< centerCoordinates_ENU.X()<<","<< centerCoordinates_ENU.Y() << "]\n";
        auto lat_lon = reproject(centerCoordinates_ENU,ref_lat,ref_lon,ref_alt);
        //gzdbg << "Thermal Lat Lon : [" << lat_lon.first << ", "<<  lat_lon.second << "]\n";
        centerCoordinates_NED = project(lat_lon.first, lat_lon.second,ref_alt,lat_home_, lon_home_);
      }

      double radius = thermal["radius"].asDouble();
      double max_strength = thermal["max_strength"].asDouble();
      double spawn_time = thermal["spawn_time"].asDouble();
      double rise_time_factor = thermal["eta"].asDouble();
      double active_period = thermal["active_period"].asDouble();
      //gzdbg << "Adding thermal at NED: ["<< centerCoordinates_NED.X()<<","<< centerCoordinates_NED.Y() << "]\n";

      thermal_manager_.addThermal(centerCoordinates_NED,radius,max_strength,spawn_time,rise_time_factor,active_period);
    }

  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
#endif

}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif
  if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
    return;
  }
  last_time_ = now;

  // Calculate the wind force.
  // Get normal distribution wind strength
  double wind_strength = std::abs(wind_velocity_distribution_(wind_velocity_generator_));
  wind_strength = (wind_strength > wind_velocity_max_) ? wind_velocity_max_ : wind_strength;
  // Get normal distribution wind direction
  ignition::math::Vector3d wind_direction;
  wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
  wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
  wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);
  // Calculate total wind velocity
  ignition::math::Vector3d wind = wind_strength * wind_direction;

  ignition::math::Vector3d wind_gust(0, 0, 0);
  // Calculate the wind gust velocity.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    // Get normal distribution wind gust strength
    double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
    wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_ : wind_gust_strength;
    // Get normal distribution wind gust direction
    ignition::math::Vector3d wind_gust_direction;
    wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
    wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
    wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
    wind_gust = wind_gust_strength * wind_gust_direction;
  }

  // Calculate wind from thermal updrafts
  thermal_manager_.UpdateTime(now.Double());

  auto wind_thermal = thermal_manager_.getWind(position_);

  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(wind.X() + wind_gust.X());
  wind_v->set_y(wind.Y() + wind_gust.Y());
  wind_v->set_z(wind.Z() + wind_gust.Z() + wind_thermal.Z());

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  wind_pub_->Publish(wind_msg);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);

void GazeboWindPlugin::GroundtruthCallback(GtPtr& groundtruth_msg) {
  auto lat_rad = groundtruth_msg->latitude_rad();
  auto lon_rad = groundtruth_msg->longitude_rad();
  auto alt_rad = groundtruth_msg->altitude();

  position_ = project(lat_rad, lon_rad, alt_rad, lat_home_, lon_home_);
}

}


