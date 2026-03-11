#include "subsystems/Hood.h"
#include <cmath>
#include "iostream"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace rev::spark;

Hood::Hood()
    : m_motor(kMotorID, SparkMax::MotorType::kBrushless),
      m_closedLoop(m_motor.GetClosedLoopController()),
      m_encoder(m_motor.GetEncoder())
{
    SparkMaxConfig config;

    config
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(40);

    // PID para posición
    config.closedLoop
        .P(0.8)   // Ajustar en práctica
        .I(0.0)
        .D(0.0)
        .OutputRange(-0.5, 0.5);

    m_motor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    m_encoder.SetPosition(0.0);
    // m_pid.SetTolerance(10.0);
    // m_pid.EnableContinuousInput(0, 360);

    frc::SmartDashboard::PutNumber("Hood Angle", GetPositionDegrees());
    frc::SmartDashboard::PutNumber("Hood Absolute Raw", m_absoluteEncoder.Get());
    frc::SmartDashboard::PutNumber("Hood AtSetpoint", m_pid.AtSetpoint());
}

void Hood::Periodic() {
    // double output = m_pid.Calculate(GetPositionDegrees(), m_targetAngle);

    // output = std::clamp(output, -0.4, 0.4);
    // if (m_pid.AtSetpoint())
    // {
    //     output = 0.0;
    // }
    // m_motor.Set(output);
}

void Hood::SetAngleRotations(double rotations)
{
    m_closedLoop.SetSetpoint(
        rotations,
        SparkMax::ControlType::kPosition
    );
}

void Hood::SetAngleDegrees(double degrees)
{
    m_targetAngle = degrees;

    // if ((GetPositionDegrees() <= degrees + 10.0) && (GetPositionDegrees() >+ degrees - 10.0))
    // {
    //     m_motor.Set(0.0);
    // }
    // else
    // {
    //     m_motor.Set(0.4);
    // }

    SetAngleRotations(DegreesToRotations(degrees));
}

void Hood::Stop()
{
    m_motor.Set(0.0);
}

double Hood::GetPositionRotations() const
{
    return m_encoder.GetPosition();
}

double Hood::GetPositionDegrees() const
{
    // double angle = ((m_absoluteEncoder.Get() * 360.0) - kEncoderOffset) / kGearRatio;
    // // angle = std::fmod(angle + 360, 360.0);
    // // if (angle < 0)
    // //     angle += 360.0;

    // return angle;

    return GetPositionRotations() * kDegreesPerRotation;

}

double Hood::DegreesToRotations(double degrees) const
{
    return degrees / kDegreesPerRotation;
}

/*
 * EJEMPLO SIMPLE distancia → ángulo
 * Luego puedes cambiarlo por interpolación real
 */
void Hood::SetAngleFromDistance(double distanceMeters)
{
    double angle;

    if (distanceMeters < 2.0)
        angle = 20.0;
    else if (distanceMeters < 3.0)
        angle = 28.0;
    else if (distanceMeters < 4.0)
        angle = 35.0;
    else
        angle = 40.0;

    SetAngleDegrees(angle);
}