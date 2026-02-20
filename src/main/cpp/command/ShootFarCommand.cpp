#include "commands/ShootFarCommand.h"
#include "subsystems/ShooterSubsystem.h"
#include <frc2/command/SubsystemBase.h>
#include <cmath>

ShootFarCommand::ShootFarCommand(ShooterSubsystem* shooter)
: m_shooter(shooter) {
    AddRequirements({m_shooter});
}

void ShootFarCommand::Initialize() {
    m_shooter->ShootFar();
}

void ShootFarCommand::End(bool interrupted) {
    m_shooter->Stop();

}


bool ShootFarCommand::IsFinished() {
    return false; 
}
