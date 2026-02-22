#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

// #include "subsystems/Shooter.h"
#include "subsystems/Transfer.h"
#include "subsystems/Feeder.h"

class ShootCommand
    : public frc2::CommandHelper<frc2::Command, ShootCommand> {

public:
    ShootCommand(
        // Shooter* shooter,
        Transfer* transfer,
        Feeder* feeder);

    void Execute() override;
    void End(bool interrupted) override;

private:
    // Shooter* m_shooter;
    Transfer* m_transfer;
    Feeder* m_feeder;
};