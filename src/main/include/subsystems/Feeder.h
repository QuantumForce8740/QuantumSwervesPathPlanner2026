#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

class Feeder : public frc2::SubsystemBase {
public:
    Feeder();

    void RunForward();   // Empujar hacia shooter
    void RunReverse();   // Regresar / desatascar
    void Stop();

private:
    static constexpr int kMotorID = 23; // Cambiar al verdadero

    static constexpr double kForwardSpeed = 0.8;
    static constexpr double kReverseSpeed = -0.6;

    SparkMax m_motor;
};