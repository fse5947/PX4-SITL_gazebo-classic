#include "thermals/thermal_manager.h"

using namespace std;

bool IsDead(boost::shared_ptr<Thermal> thermal) { return thermal->isOver(); }


void ThermalManager::UpdateTime(double time) {

    SpawnThermals(time);
    for (auto& thermal : active_thermals_){
        thermal->UpdateTime(time);
    }
    // https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
    auto prev_size = active_thermals_.size();
    active_thermals_.erase(remove_if(active_thermals_.begin(), active_thermals_.end(), IsDead), active_thermals_.end());
    auto nbr_deletions = prev_size - active_thermals_.size();
    // if (nbr_deletions > 0)
    //     cout << '[' << int(time) << ']' << " Removed: " << nbr_deletions << " thermals, now " << active_thermals_.size() << " thermals\n";
}

void ThermalManager::addThermal(ignition::math::Vector3d &center, double &strength, double &radius) {
    thermals_.push_back(boost::shared_ptr<Thermal>(new ConstantThermal(center, strength, radius)));

    if (thermals_.size() == 1) {
        next_to_spawn_ = thermals_.begin();
    }
}

void ThermalManager::addThermal(ignition::math::Vector3d &center, double &radius, double &max_strength, double &spawn_time, double &rise_time_factor, double &active_period) {
    thermals_.push_back(boost::shared_ptr<Thermal>(new DynamicThermal(center, radius, max_strength, spawn_time, rise_time_factor, active_period)));

    if (thermals_.size() == 1) {
        next_to_spawn_ = thermals_.begin();
    }
}

ignition::math::Vector3d ThermalManager::getWind(ignition::math::Vector3d &position){
    auto wind = ignition::math::Vector3d(0.0,0.0,env_sink_);

    bool in_thermal = false;
    for (auto& thermal : active_thermals_){
        if (thermal->inRange(position)){
            wind += thermal->getWind(position);
            in_thermal = true;
        }
    }
    if (!in_thermal_ && in_thermal){
        gzmsg << "Entered Thermal!\n";
    }
    if (in_thermal_ && !in_thermal){
        gzmsg << "Left Thermal!\n";
    }
    in_thermal_ = in_thermal;
    return wind;
}

void ThermalManager::SpawnThermals(double &time){
    while ((next_to_spawn_ != thermals_.end()) && (*next_to_spawn_)->ShouldSpawn(time)) {
        active_thermals_.push_back(*next_to_spawn_);
        //cout << '[' << int(time) << ']' << " Spawning Thermal, now " << active_thermals_.size() << " thermals\n";
        next_to_spawn_++;
    }
}