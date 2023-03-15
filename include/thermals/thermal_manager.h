/**
 * @brief Thermal Manager
 *
 * Thermal Manager class
 *
 * @author Alexandre Borowczyk <alexandre.borowczyk@shearwater.ai>
 *
 */

#ifndef _THERMAL_MANAGER_HH_
#define _THERMAL_MANAGER_HH_

#include <common.h>
#include <ignition/math.hh>

#include "thermals/thermals.h"

class ThermalManager {
public:
    ThermalManager(double env_sink): env_sink_(env_sink) {};
    virtual ~ThermalManager() {};
    virtual void UpdateTime(double time);
    virtual void addThermal(ignition::math::Vector3d &center, double &strength, double &radius);
    virtual void addThermal(ignition::math::Vector3d &center, double &radius, double &max_strength, double &spawn_time, double &rise_time_factor, double &period);
    virtual ignition::math::Vector3d getWind(ignition::math::Vector3d &position);

private:
    virtual void SpawnThermals(double &time);

    std::list<boost::shared_ptr<Thermal>> thermals_{}; // List so the the iterator remain valid if new thermals are inserted
    std::vector<boost::shared_ptr<Thermal>> active_thermals_{};
    std::list<boost::shared_ptr<Thermal>>::iterator next_to_spawn_ = thermals_.end();

    double env_sink_{0.0};
};
#endif // _THERMAL_MANAGER_HH_