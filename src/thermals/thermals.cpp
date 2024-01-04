
#include "thermals/thermals.h"

using namespace std;

ConstantThermal::ConstantThermal(ignition::math::Vector3d &center, double &strength, double &radius) : center_{center},
                                                                                                       strength_{strength},
                                                                                                       radius_{radius}
{
}

void ConstantThermal::UpdateTime(double &time)
{
}
bool ConstantThermal::inRange(ignition::math::Vector3d &position)
{
    return (distance_from(position, center_) < (3.0 * radius_));
}
bool ConstantThermal::isOver()
{
    return false;
}
bool ConstantThermal::ShouldSpawn(double &time)
{
    std::cout << "Should spawn\n";
    return true;
}
double ConstantThermal::getNetUpdraft()
{
    return (strength_ * 9.0 * M_PI * pow(radius_,2.0) / exp(9.0));
}
ignition::math::Vector3d ConstantThermal::getWind(ignition::math::Vector3d &position)
{

    double distance_from_center = distance_from(position, center_);

    double dist_ratio = pow(distance_from_center / radius_, 2.0);

    double wind_u = strength_ * exp(-dist_ratio) * (1 - dist_ratio);

    //std::cout << "Distance: " << distance_from_center << " Wind Up: " << wind_u << '\n';

    return ignition::math::Vector3d(0.0, 0.0, wind_u);
}

DynamicThermal::DynamicThermal(ignition::math::Vector3d &center, double &radius, double &max_strength, double &spawn_time, double &rise_time_factor, double &active_period): ConstantThermal(center,max_strength,radius),
                                                                                                                                                               max_strength_{max_strength},
                                                                                                                                                               spawn_time_{spawn_time},
                                                                                                                                                               rise_time_factor_{rise_time_factor},
                                                                                                                                                               active_period_{active_period}
{
    double threshold = 0.01;
    duration_ = (((log((1.0 / threshold) - 1)) / rise_time_factor_) + (0.5 * active_period_)) * 2.0;
}

void DynamicThermal::UpdateTime(double &time)
{
    lifetime_ = time - spawn_time_;
}

ignition::math::Vector3d DynamicThermal::getWind(ignition::math::Vector3d &position)
{
    if (lifetime_ < 0.0)
        return ignition::math::Vector3d(0.0, 0.0, 0.0);

    auto decay_exp = 1.0 / (exp(rise_time_factor_ * (lifetime_ - 0.5 * (duration_ + active_period_))) + 1.0);
    auto raise_exp = 1.0 / (exp(rise_time_factor_ * (0.5 * (duration_ - active_period_) - lifetime_)) + 1.0);
    strength_ = (decay_exp + raise_exp - 1.0) * max_strength_;

    return ConstantThermal::getWind(position);
}

double DynamicThermal::getNetUpdraft()
{
    return (strength_ * 9.0 * M_PI * pow(radius_,2.0) / exp(9.0));
}

bool DynamicThermal::isOver()
{
    return lifetime_ > duration_;
}
bool DynamicThermal::ShouldSpawn(double &time)
{
    return time >= spawn_time_;
}
