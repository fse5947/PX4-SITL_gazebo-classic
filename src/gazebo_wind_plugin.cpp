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

  // Get thermal updraft params from SDF
  if (sdf->HasElement("Thermals")) {
		sdf::ElementPtr thermals = sdf->GetElement("Thermals");
		sdf::ElementPtr thermal = thermals->GetFirstElement();

		ignition::math::Vector3d centerCoordinates_DEG = ignition::math::Vector3d(0.0, 0.0, 0.0);
    double thermal_strength_{0.0};
    double thermal_radius_{50.0};

    double lat_home_ = kDefaultHomeLatitude;
    double lon_home_ = kDefaultHomeLongitude;

		while( thermal != nullptr && thermal->HasElement("centerCoordinates")) {

			using_thermal_ = true;

			getSdfParam<ignition::math::Vector3d>(thermal, "centerCoordinates", centerCoordinates_DEG, centerCoordinates_DEG);
      getSdfParam<double>(thermal, "ThermalStrength", thermal_strength_, thermal_strength_);
      getSdfParam<double>(thermal, "ThermalRadius", thermal_radius_, thermal_radius_);

      std::cout << centerCoordinates_DEG << std::endl;

			const double lat_rad = centerCoordinates_DEG.X() * M_PI / 180.0;
			const double lon_rad = centerCoordinates_DEG.Y() * M_PI / 180.0;
			const double sin_lat = sin(lat_rad);
			const double cos_lat = cos(lat_rad);
			const double ref_sin_lat = sin(lat_home_);
			const double ref_cos_lat = cos(lat_home_);
			const double cos_d_lon = cos(lon_rad - lon_home_);
			const double arg = fmin(fmax(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0),  1.0);
			const double c = acos(arg);
			double k = 1.0;

			if (fabs(c) > 0) {
				k = (c / sin(c));
			}

			double x = static_cast<float>(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * earth_radius);
			double y = static_cast<float>(k * cos_lat * sin(lon_rad - lon_home_) * earth_radius);

			ignition::math::Vector3d centerCoordinates_ENU(y,x,0.0);

			thermal_centers_.push_back(centerCoordinates_ENU);
      thermal_strengths_.push_back(thermal_strength_);
      thermal_radii_.push_back(thermal_radius_);

			gzdbg << "Adding thermal at lat: "<< centerCoordinates_DEG.X()<<", lon: "<< centerCoordinates_DEG.Y() << "], ENU: ["<< centerCoordinates_ENU.X()<<","<< centerCoordinates_ENU.Y() << "]\n";

			thermal = thermal->GetNextElement();
		}

	} else {
		gzdbg << "[gazebo_wind_plugin] No Thermals.\n";
		using_thermal_ = false;
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

  ignition::math::Vector3d wind_thermal(0, 0, 0); // Wind speed from thermal

  // Distances from thermal
	double thermal_dist = -1.0;
	double dist_x = -1.0;
	double dist_y = -1.0;
	int closest_thermal = -1;
	for (int i = 0; i < thermal_centers_.size(); i++) {
		ignition::math::Vector3d thermal_center = thermal_centers_.at(i);
		double current_dist_x = fabs(thermal_center.X() - gps_x);
		double current_dist_y = fabs(thermal_center.Y() - gps_y);

		double current_thermal_dist = sqrt(current_dist_x * current_dist_x + current_dist_y * current_dist_y);

		if (i == 0 || thermal_dist > current_thermal_dist) {
			closest_thermal = i;
			thermal_dist = current_thermal_dist;
			dist_x = current_dist_x;
			dist_y = current_dist_y;
		}
	}

	if (thermal_dist < 0.0 || dist_x < 0.0 || dist_y < 0.0) {
		gzerr << "Invalid Thermal distance \n";
	}

  std::cout << "Distance from thermal: " << thermal_dist << std::endl;

  if (thermal_dist < 3 * thermal_radii_.at(closest_thermal)) {
    wind_thermal.Z() = thermal_strengths_.at(closest_thermal) *
                    exp(-pow(thermal_dist/thermal_radii_.at(closest_thermal),2.0)) *
                    (1 - pow(thermal_dist/thermal_radii_.at(closest_thermal),2.0));

  if (!in_thermal_) {
      gzmsg << "Entered Thermal #"<< closest_thermal << "\n";
      in_thermal_ = true;
    }

  } else if (in_thermal_) {
    gzmsg << "Left Thermal\n";
    in_thermal_ = false;
  }

  std::cout << wind_thermal.Z() << std::endl;

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
  // receive groundtruth lat_rad, lon_rad and altitude from gps plugin
  groundtruth_lat_rad_ = groundtruth_msg->latitude_rad();
  groundtruth_lon_rad_ = groundtruth_msg->longitude_rad();
  groundtruth_altitude_ = groundtruth_msg->altitude();

  double lat_home_ = kDefaultHomeLatitude;
  double lon_home_ = kDefaultHomeLongitude;

  const double sin_lat = sin(groundtruth_lat_rad_);
  const double cos_lat = cos(groundtruth_lat_rad_);
  const double ref_sin_lat = sin(lat_home_);
  const double ref_cos_lat = cos(lat_home_);
  const double cos_d_lon = cos(groundtruth_lon_rad_ - lon_home_);
  const double arg = fmin(fmax(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0),  1.0);
  const double c = acos(arg);
  double k = 1.0;

  if (fabs(c) > 0) {
    k = (c / sin(c));
  }

  gps_x = static_cast<float>(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * earth_radius);
  gps_y = static_cast<float>(k * cos_lat * sin(groundtruth_lon_rad_ - lon_home_) * earth_radius);
}

}


