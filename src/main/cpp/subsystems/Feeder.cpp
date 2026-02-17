#include "subsystems/Feeder.h"

using namespace rev::spark;

Feeder::Feeder()
    : m_motor(kMotorID, SparkMax::MotorType::kBrushless)
{
    SparkMaxConfig config;

    config
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);

    m_motor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );
}

void Feeder::RunForward()
{
    m_motor.Set(kForwardSpeed);
}

void Feeder::RunReverse()
{
    m_motor.Set(kReverseSpeed);
}

void Feeder::Stop()
{
    m_motor.Set(0.0);
}