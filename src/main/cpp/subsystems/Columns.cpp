#include "subsystems/Columns.h"

using namespace rev;
using namespace rev::spark;

Columns::Columns()
    : m_motor(kMotorID, SparkMax::MotorType::kBrushless)
{
    SparkMaxConfig config;

    config
        .Inverted(false) // Cambiar si gira al revés
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);

    m_motor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters
    );
}

void Columns::RunForward()
{
    m_motor.Set(kForwardSpeed);
}

void Columns::RunReverse()
{
    m_motor.Set(kReverseSpeed);
}

void Columns::Stop()
{
    m_motor.Set(0.0);
}