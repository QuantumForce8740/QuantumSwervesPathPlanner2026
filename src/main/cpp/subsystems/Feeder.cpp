#include "subsystems/Feeder.h"

using namespace ctre::phoenix6;

Feeder::Feeder()
    : m_motor(kMotorID)
{
    configs::TalonFXConfiguration config{};

    config.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive; // Cambiar si gira al revés
    config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    config.CurrentLimits.SupplyCurrentLimit = 40_A; //qie significa A 
    config.CurrentLimits.SupplyCurrentLimitEnable = true; 
     
    m_motor.GetConfigurator().Apply(config);
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