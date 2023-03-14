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


inline double distance_from(ignition::math::Vector3d &position, ignition::math::Vector3d &center) {
    auto distance = (position - center);
    distance.Z() = 0.0;
	return distance.Length();
}

class Thermal {
public:
    Thermal() {};
    virtual ~Thermal() {};
    virtual void UpdateTime(double &time) = 0;
    virtual bool inRange(ignition::math::Vector3d &position) = 0;
    virtual bool ShouldSpawn(double &time) = 0;
    virtual bool isAlive() = 0;
    virtual ignition::math::Vector3d getWind(ignition::math::Vector3d &position) = 0;
};

class ConstantThermal : public Thermal {
public:
    ConstantThermal(ignition::math::Vector3d &center, double &strength, double &radius);
    virtual ~ConstantThermal() {};
    virtual void UpdateTime(double &time);
    virtual bool inRange(ignition::math::Vector3d &position);
    virtual bool ShouldSpawn(double &time);
    virtual bool isAlive();
    virtual ignition::math::Vector3d getWind(ignition::math::Vector3d &position);

private:
    ignition::math::Vector3d center_ = ignition::math::Vector3d(0.0,0.0,0.0);
    double strength_{1};
    double radius_{1};

};
#endif // _THERMAL_MODEL_HH_
