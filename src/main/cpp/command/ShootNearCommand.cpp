#pragma once

#include <frc2/command/SubsystemBase.h>
#include "subsystems/ShooterSubsystem.h"
#include <cmath> 
#include "commands/ShootNearCommand.h"

ShootNearCommand::ShootNearCommand(ShooterSubsystem* shooter)
: m_shooter(shooter) {
    AddRequirements(m_shooter);
}

void ShootNearCommand::Initialize() {

    m_shooter->ShootNear();
}
bool ShootNearCommand::IsFinished()
{
 return false; 
}
