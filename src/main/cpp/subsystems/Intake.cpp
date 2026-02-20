#include "subsystems/Intake.h" 

using namespace rev::spark;

IntakeSubsystem::IntakeSubsystem() :
    m_rollerMotor(kRollerMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    m_deployMotor(kDeployMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless)
{
    SparkMaxConfig config;

    config
        .Inverted(false) // Cambiar si gira al revés
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);

    m_rollerMotor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    m_deployMotor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );
}

void IntakeSubsystem::Deploy() {
    m_deployMotor.Set(0.4);
}

void IntakeSubsystem::Retract() {
    m_deployMotor.Set(-0.4);
}

void IntakeSubsystem::IntakeIn() {
    m_rollerMotor.Set(0.8);
}

void IntakeSubsystem::IntakeOut() {
    m_rollerMotor.Set(-0.8);
}

void IntakeSubsystem::StopRollers() {
    m_rollerMotor.Set(0);
}