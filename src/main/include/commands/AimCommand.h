#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include "subsystems/Vision.h"
// #include "subsystems/Shooter.h"
#include "subsystems/Hood.h"
#include "subsystems/CommandSwerveDrivetrain.h"

class AimCommand
    : public frc2::CommandHelper<frc2::Command, AimCommand> {

public:
    AimCommand(
        Vision* vision,
        // Shooter* shooter,
        Hood* hood,
        subsystems::CommandSwerveDrivetrain* drive);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;

private:
    Vision* m_vision;
    // Shooter* m_shooter;
    Hood* m_hood;
    subsystems::CommandSwerveDrivetrain* m_drive;
    frc::PIDController m_turnPID{0.08, 0.0, 0.0008};
};