#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6;

class Feeder : public frc2::SubsystemBase {
public:
    Feeder();

    void RunForward();   // Empujar hacia shooter
    void RunReverse();   // Regresar / desatascar
    void Stop();

private:
    static constexpr int kMotorID = 24; // Cambiar al verdadero

    static constexpr double kForwardSpeed = 0.8;
    static constexpr double kReverseSpeed = -0.6;


    ctre::phoenix6::hardware::TalonFX m_motor;
    ctre::phoenix6::controls::VoltageOut m_voltageOut {0_V}; 
};