#include "subsystems/Transfer.h"

using namespace rev::spark;

Transfer::Transfer()
    : m_motor(kMotorID, SparkMax::MotorType::kBrushless)
{
    SparkMaxConfig config;

    config
        .Inverted(true)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(40);

    m_motor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );
}

void Transfer::RunForward()
{
    m_motor.Set(kForwardSpeed);
}

void Transfer::RunReverse()
{
    m_motor.Set(kReverseSpeed);
}

void Transfer::Stop()
{
    m_motor.Set(0.0);
}