#include "subsystems/ShooterSubsystem.h"
#include <units/velocity.h>
#include <frc/Encoder.h>
#include <cmath>

using namespace rev::spark;

ShooterSubsystem::ShooterSubsystem() :
    m_motor1(20, SparkMax::MotorType::kBrushless),
    m_motor2(21, SparkMax::MotorType::kBrushless),
    m_motor3(22, SparkMax::MotorType::kBrushless),
    //m_closedLoop1(m_motor1.GetClosedLoopController()),
    //m_closedLoop2(m_motor2.GetClosedLoopController()),
    //m_closedLoop3(m_motor3.GetClosedLoopController()),
    m_PID1(m_motor1.GetClosedLoopController()),
    m_PID2(m_motor2.GetClosedLoopController()),
    m_PID3(m_motor3.GetClosedLoopController()),
    // Initialize encoders from the motors (use the motor's GetEncoder())
    m_encoder1(m_motor1.GetEncoder()),
    m_encoder2(m_motor2.GetEncoder()),
    m_encoder3(m_motor3.GetEncoder())
    // Posiblemente util m_targetRPM(0.0)

{
    SparkMaxConfig config;
    config.Inverted(false)
          .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
          .SmartCurrentLimit(50);

    //PID position control 
    config.closedLoop
    .P(0.8)
    .I(0.0)
    .D(0.0)
    .VelocityFF(0.00018)//Cambiado por SetFF
    .OutputRange(-0.5, 0.5);

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

void ShooterSubsystem::SetRPM(double rpm) {
    m_targetRPM = rpm;
    m_PID1.SetReference(rpm, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID2.SetReference(rpm, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID3.SetReference(rpm, rev::spark::SparkMax::ControlType::kVelocity);
}

void ShooterSubsystem::Stop() {
    SetRPM(0);
}
// body: configure inversion, PID values, etc.
bool ShooterSubsystem::AtSetpoint() {
    return
        fabs(m_encoder1.GetVelocity() - m_targetRPM) < 100 &&
        fabs(m_encoder2.GetVelocity() - m_targetRPM) < 100 &&
        fabs(m_encoder3.GetVelocity() - m_targetRPM) < 100;
}

double ShooterSubsystem::getRPM() {
    return (m_encoder1.GetVelocity() + m_encoder2.GetVelocity() + m_encoder3.GetVelocity()) / 3.0;
}