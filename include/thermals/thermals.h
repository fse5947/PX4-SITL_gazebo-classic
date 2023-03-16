/**
 * @brief Thermal Model
 *
 * Thermal model class
 *
 * @author Alexandre Borowczyk <alexandre.borowczyk@shearwater.ai>
 *
 */

#ifndef _THERMAL_MODEL_HH_
#define _THERMAL_MODEL_HH_

#include <common.h>
#include <ignition/math.hh>

inline double distance_from(ignition::math::Vector3d &position, ignition::math::Vector3d &center)
{
    auto distance = (position - center);
    distance.Z() = 0.0;
    return distance.Length();
}

class Thermal
{
public:
    Thermal(){};
    virtual ~Thermal(){};
    virtual void UpdateTime(double &time) = 0;
    virtual bool inRange(ignition::math::Vector3d &position) = 0;
    virtual bool ShouldSpawn(double &time) = 0;
    virtual bool isOver() = 0;
    virtual ignition::math::Vector3d getWind(ignition::math::Vector3d &position) = 0;
};

class ConstantThermal : public Thermal
{
public:
    ConstantThermal(ignition::math::Vector3d &center, double &strength, double &radius);
    virtual ~ConstantThermal(){};
    virtual void UpdateTime(double &time);
    virtual bool inRange(ignition::math::Vector3d &position);
    virtual bool ShouldSpawn(double &time);
    virtual bool isOver();
    virtual ignition::math::Vector3d getWind(ignition::math::Vector3d &position);

protected:
    ignition::math::Vector3d center_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
    double strength_{1};
    double radius_{1};
};

class DynamicThermal : public ConstantThermal
{
public:
    DynamicThermal(ignition::math::Vector3d &center, double &radius, double &max_strength, double &spawn_time, double &rise_time, double &active_period);
    virtual ~DynamicThermal(){};
    virtual void UpdateTime(double &time);
    virtual bool ShouldSpawn(double &time);
    virtual bool isOver();
    virtual ignition::math::Vector3d getWind(ignition::math::Vector3d &position);

private:
    double max_strength_{0.0};
    double spawn_time_{0.0};
    double rise_time_factor_{0.0};
    double active_period_{0.0};

    double duration_{0.0};
    double lifetime_{0.0};
};
#endif // _THERMAL_MODEL_HH_
