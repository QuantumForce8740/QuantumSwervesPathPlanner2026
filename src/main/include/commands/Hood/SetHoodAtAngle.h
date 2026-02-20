#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "subsystems/Hood.h"

class SetHoodAtAngle
    : public frc2::CommandHelper<frc2::Command, SetHoodAtAngle> {

public:
    SetHoodAtAngle(Hood* hood, double angleDegrees);

    void Initialize() override;
    bool IsFinished() override;

private:
    Hood* m_hood;
    double m_targetAngle;
};