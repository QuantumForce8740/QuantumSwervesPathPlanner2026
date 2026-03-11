#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include "subsystems/HubVision.h"
#include "subsystems/Shooter.h"
#include "subsystems/Hood.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "ShooterLookupTable.h"

class AimCommand
    : public frc2::CommandHelper<frc2::Command, AimCommand> {

public:
    AimCommand(
        HubVision* hubVision,
        Shooter* shooter,
        Hood* hood);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;

private:
    HubVision* m_hubVision;
    Shooter* m_shooter;
    Hood* m_hood;
};