#include "subsystems/Hood.h"
#include <cmath>

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
        .SmartCurrentLimit(50);

    // PID para posición
    config.closedLoop
        .P(0.8)   // ⚠️ Ajustar en práctica
        .I(0.0)
        .D(0.0)
        .OutputRange(-0.5, 0.5);

    m_motor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );

    m_encoder.SetPosition(0.0);
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