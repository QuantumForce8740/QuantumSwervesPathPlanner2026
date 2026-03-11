#include "subsystems/Intake.h" 

using namespace rev;
using namespace rev::spark;

Intake::Intake() :
    m_rollerMotor(kRollerMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    m_deployMotor(kDeployMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    m_deployClosedLoop(m_deployMotor.GetClosedLoopController()),
    m_deployEncoder(m_deployMotor.GetEncoder())
{
    SparkMaxConfig rollerConfig;
    SparkMaxConfig deployConfig;

    rollerConfig
        .Inverted(true) // Cambiar si gira al revés
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(40);

    m_rollerMotor.Configure(
        rollerConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    deployConfig
        .Inverted(true) // Cambiar si gira al revés
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(40);

    deployConfig.closedLoop
        .P(0.05)   // Ajustar en práctica
        .I(0.0)
        .D(0.0)
        .OutputRange(-0.5, 0.5);

    m_deployMotor.Configure(
        deployConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    m_deployEncoder.SetPosition(0.0); // Home inicial
    m_targetAngle = 0.0;

    frc::SmartDashboard::PutNumber("Intake Angle", GetAngleDegrees());
    frc::SmartDashboard::PutNumber("Intake At Setpoint", AtSetpoint());
}

void Intake::SetDeployPositionRotations(double rotations)
{
    m_deployClosedLoop.SetSetpoint(
        rotations,
        SparkMax::ControlType::kPosition
    );
}

void Intake::SetAngleDegrees(double degrees)
{
    m_targetAngle = degrees;
    double motorRotations = DegreesToMotorRotations(degrees);

    m_deployClosedLoop.SetSetpoint(
        motorRotations,
        SparkMax::ControlType::kPosition
    );
}

void Intake::Deploy()
{
    SetAngleDegrees(kDeployAngle);
}

void Intake::Retract()
{
    SetAngleDegrees(kRetractAngle);
}

double Intake::GetAngleDegrees() const
{
    return MotorRotationsToDegrees(m_deployEncoder.GetPosition());
}

double Intake::DegreesToMotorRotations(double degrees) const
{
    return (degrees / 360.0) * kGearRatio;
}

double Intake::MotorRotationsToDegrees(double rotations) const
{
    return (rotations / kGearRatio) * 360.0;
}

bool Intake::AtSetpoint(){
    return std::abs(GetAngleDegrees() - m_targetAngle) < 10;
}

void Intake::IntakeIn() {
    m_rollerMotor.Set(0.8);
}

void Intake::IntakeOut() {
    m_rollerMotor.Set(-0.8);
}

void Intake::StopRollers() {
    m_rollerMotor.Set(0);
}