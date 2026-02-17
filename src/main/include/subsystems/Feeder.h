#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

class Feeder : public frc2::SubsystemBase {
public:
    Feeder();

    void RunForward();     // Llevar fuel hacia shooter
    void RunReverse();     // Regresar fuel (desatascar)
    void Stop();

private:
    static constexpr int kMotorID = 20;
    static constexpr double kForwardSpeed = 0.7;
    static constexpr double kReverseSpeed = -0.5;

    SparkMax m_motor;
};