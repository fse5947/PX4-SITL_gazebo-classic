
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
bool ConstantThermal::isAlive()
{
    return true;
}
bool ConstantThermal::ShouldSpawn(double &time)
{
    std::cout << "Should spawn\n";
    return true;
}
ignition::math::Vector3d ConstantThermal::getWind(ignition::math::Vector3d &position)
{

	double distance_from_center = distance_from(position, center_);

    std::cout << "dist " << distance_from_center << " radius " << radius_ << '\n';

    double dist_ratio = pow(distance_from_center / radius_, 2.0);

    return ignition::math::Vector3d(0.0, 0.0, strength_ * exp(-dist_ratio) * (1 - dist_ratio));
}