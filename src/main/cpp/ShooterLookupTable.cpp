#include "ShooterLookupTable.h"

std::map<double, ShotParameters> ShooterLookupTable::table = {

    {1.5, {2500, 18}},
    {2.0, {2700, 22}},
    {2.5, {3000, 27}},
    {3.0, {3300, 32}},
    {3.5, {3600, 36}},
    {4.0, {3900, 40}}

};

ShotParameters ShooterLookupTable::GetShot(double distance)
{
    auto upper = table.lower_bound(distance);

    if (upper == table.begin())
        return upper->second;

    if (upper == table.end())
        return std::prev(upper)->second;

    auto lower = std::prev(upper);

    double d1 = lower->first;
    double d2 = upper->first;

    auto s1 = lower->second;
    auto s2 = upper->second;

    double t = (distance - d1) / (d2 - d1);

    ShotParameters result;

    result.rpm =
        s1.rpm + t * (s2.rpm - s1.rpm);

    result.hoodAngle =
        s1.hoodAngle + t * (s2.hoodAngle - s1.hoodAngle);

    return result;
}
