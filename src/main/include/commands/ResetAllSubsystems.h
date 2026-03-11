#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include "subsystems/Shooter.h"
#include "subsystems/Hood.h"
#include "subsystems/Transfer.h"
#include "subsystems/Feeder.h"
#include "subsystems/CommandSwerveDrivetrain.h"

class ResetAllSubsystems
    : public frc2::CommandHelper<frc2::Command, ResetAllSubsystems> {

public:
    ResetAllSubsystems(
        Shooter* shooter,
        Hood* hood,
        Transfer* transfer, 
        Feeder* feeder);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;

private:
    Shooter* m_shooter;
    Hood* m_hood;
    Transfer* m_transfer;
    Feeder* m_feeder;
};