#include "subsystems/Feeder.h"

using namespace ctre::phoenix6; 

Feeder::Feeder()
    : m_motor(kMotorID)
{
    configs::TalonFXConfiguration config{}; 

    config.MotorOutput.Inverted =
    signals::InvertedValue::Clockwise_Positive; // Cambiar si gira al revés
    config.MotorOutput.NeutralMode
     = signals::NeutralModeValue::Brake;

     config.CurrentLimits.SupplyCurrentLimit = 50_A; //qie significa A 
     config.CurrentLimits.SupplyCurrentLimitEnable = true; 
     
         m_motor.GetConfigurator().Apply(config);
    }
    
         // SupplyCurrentLimit is not directly assignable; use SmartCurrentLimit below.

void Feeder::RunForward()
{
 m_voltageOut.Output = kForwardSpeed * 12.0_V;   
 m_motor.SetControl(m_voltageOut);
}

void Feeder::RunReverse()
{
   m_voltageOut.Output = kReverseSpeed * 12.0_V;
   m_motor.SetControl(m_voltageOut);
}

void Feeder::Stop()
{
 m_voltageOut.Output = 0_V;
 m_motor.SetControl(m_voltageOut);
}