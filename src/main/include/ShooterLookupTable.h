#pragma once

#include <map>

struct ShotParameters
{
    double rpm;
    double hoodAngle;
};

class ShooterLookupTable
{
public:

    static ShotParameters GetShot(double distance);

private:

    static std::map<double, ShotParameters> table;

};
