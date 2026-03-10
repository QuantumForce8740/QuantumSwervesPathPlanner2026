#include "ShooterLookupTable.h"

std::map<double, ShotParameters> ShooterLookupTable::table = {

    {1.4, {3100, 10}},
    {1.8, {3100, 15}},
    {2.2, {3100, 19}},
    {2.6, {3400, 19}},
    {3.1, {3400, 25}},
    {3.5, {3400, 25}},
    {3.7, {3700, 25}},
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
