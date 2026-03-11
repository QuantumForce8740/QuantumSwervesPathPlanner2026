#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include "subsystems/Vision.h"
#include "subsystems/Shooter.h"
#include "subsystems/Hood.h"
#include "subsystems/CommandSwerveDrivetrain.h"

class HubAimCommand
    : public frc2::CommandHelper<frc2::Command, HubAimCommand> {

public:
    HubAimCommand(
        Shooter* shooter,
        Hood* hood);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;

private:
    Shooter* m_shooter;
    Hood* m_hood;
};