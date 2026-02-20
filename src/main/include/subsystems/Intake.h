#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    void Deploy();
    void Retract();

    void IntakeIn();
    void IntakeOut();
    void StopRollers();

private:

    static constexpr int kRollerMotorID = 23; // Cambiar al verdadero
    static constexpr int kDeployMotorID = 24; // Cambiar al verdadero

    SparkMax m_rollerMotor;
    SparkMax m_deployMotor;
};