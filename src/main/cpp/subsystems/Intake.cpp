#include "Intake.h" 
#include <rev/CANSparkMax.h>

IntakeSubsystem::IntakeSubsystem() :
    m_rollerMotor(10, rev::spark::SparkLowLevel::MotorType::kBrushless),
    m_deployMotor(11, rev::spark::SparkLowLevel::MotorType::kBrushless)
{}

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