#include "thermals/thermal_manager.h"

using namespace std;

bool IsDead(boost::shared_ptr<Thermal> thermal) { return !thermal->isAlive(); }


void ThermalManager::UpdateTime(double time) {

    SpawnThermals(time);
    for (auto& thermal : active_thermals_){
        thermal->UpdateTime(time);
    }
    // https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom
    active_thermals_.erase(remove_if(active_thermals_.begin(), active_thermals_.end(), IsDead), active_thermals_.end());
}

void ThermalManager::addThermal(ignition::math::Vector3d &center, double &strength, double &radius) {
    thermals_.push_back(boost::shared_ptr<Thermal>(new ConstantThermal(center, strength, radius)));

    if (thermals_.size() == 1) {
        next_to_spawn_ = thermals_.begin();
    }
}

void ThermalManager::addThermal(ignition::math::Vector3d &center, double &strength, double &radius, double &spawn_time) {

}

ignition::math::Vector3d ThermalManager::getWind(ignition::math::Vector3d &position){
    auto wind = ignition::math::Vector3d(0.0,0.0,env_sink_);

    for (auto& thermal : active_thermals_){
        if (thermal->inRange(position)){
            wind += thermal->getWind(position);
        }
    }
    return wind;
}

void ThermalManager::SpawnThermals(double &time){
    while ((next_to_spawn_ != thermals_.end()) && (*next_to_spawn_)->ShouldSpawn(time)) {
        cout << "Spawning Thermal" << '\n';
        active_thermals_.push_back(*next_to_spawn_);
        next_to_spawn_++;
    }
}