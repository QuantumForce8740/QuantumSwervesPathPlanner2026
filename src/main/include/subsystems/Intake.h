#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    void Deploy();
    void Retract();

    void IntakeIn();
    void IntakeOut();
    void StopRollers();

private:
    rev::CANSparkMax m_rollerMotor;
    rev::CANSparkMax m_deployMotor;
};