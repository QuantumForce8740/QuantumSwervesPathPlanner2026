#include "subsystems/Shooter.h"
#include <units/velocity.h>
#include <frc/Encoder.h>
#include <cmath>

using namespace rev::spark;

Shooter::Shooter() :
    m_motor1(kMotorID, SparkMax::MotorType::kBrushless),
    m_motor2(kMotorID2, SparkMax::MotorType::kBrushless),
    m_motor3(kMotorID3, SparkMax::MotorType::kBrushless),

    m_PID1(m_motor1.GetClosedLoopController()),
    m_PID2(m_motor2.GetClosedLoopController()),
    m_PID3(m_motor3.GetClosedLoopController()),

    m_encoder1(m_motor1.GetEncoder()),
    m_encoder2(m_motor2.GetEncoder()),
    m_encoder3(m_motor3.GetEncoder()),
    m_targetRPM(0.0)

{
    SparkMaxConfig config;
    config.Inverted(false)
          .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
          .SmartCurrentLimit(50);

    //PID position control 
    config.closedLoop
    .P(0.0001)
    .I(0.0)
    .D(0.0)
    .VelocityFF(0.001)//Cambiado por SetFF
    .OutputRange(-1.0, 1.0);

    m_motor1.Configure(
    config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
    );

    m_motor2.Configure(
    config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
    );

    m_motor3.Configure(
    config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
    );
}

void Shooter::SetRPM(double rpm)
{
    m_targetRPM = rpm;

    m_PID1.SetReference(rpm, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID2.SetReference(rpm, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID3.SetReference(rpm, rev::spark::SparkMax::ControlType::kVelocity);
}

void Shooter::Stop()
{
    SetRPM(0.0);
}

bool Shooter::AtSetpoint()
{
    return
        fabs(m_encoder1.GetVelocity() - m_targetRPM) < kToleranceRPM &&
        fabs(m_encoder2.GetVelocity() - m_targetRPM) < kToleranceRPM &&
        fabs(m_encoder3.GetVelocity() - m_targetRPM) < kToleranceRPM;
}

double Shooter::GetRPM() const
{
    return (m_encoder1.GetVelocity()
          + m_encoder2.GetVelocity()
          + m_encoder3.GetVelocity()) / 3.0;
}

double Shooter::CalculateRPMFromDistance(double distance)
{
    // Tabla simple (ajustar con datos reales)
    const double d1 = 1.5;
    const double d2 = 2.0;
    const double d3 = 2.5;
    const double d4 = 3.0;

    const double r1 = 3200;
    const double r2 = 3600;
    const double r3 = 4000;
    const double r4 = 4400;

    // Interpolación lineal por segmentos
    if (distance <= d1) return r1;

    if (distance < d2)
        return r1 + (distance - d1) * (r2 - r1) / (d2 - d1);

    if (distance < d3)
        return r2 + (distance - d2) * (r3 - r2) / (d3 - d2);

    if (distance < d4)
        return r3 + (distance - d3) * (r4 - r3) / (d4 - d3);

    return r4;
}

void Shooter::ShootAuto(double distanceMeters)
{
    double rpm = CalculateRPMFromDistance(distanceMeters);
    SetRPM(rpm);
}