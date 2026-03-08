#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "subsystems/Intake.h"

class OscillateIntake
    : public frc2::CommandHelper<frc2::Command, OscillateIntake> {

public:
    OscillateIntake(Intake* intake);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    Intake* m_intake;
    bool m_targetUp = true;

    static constexpr double kAngleUp = 50.0;
    static constexpr double kAngleDown = 85.0;
};