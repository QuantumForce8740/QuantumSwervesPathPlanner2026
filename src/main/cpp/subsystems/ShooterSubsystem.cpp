#include "subsystems/ShooterSubsystem.h"
#include <units/velocity.h>
#include <frc/Encoder.h>
#include <cmath>

using namespace rev::spark;

ShooterSubsystem::ShooterSubsystem() :
    m_motor1(kMotorID, SparkMax::MotorType::kBrushless),
    m_motor2(kMotorID2, SparkMax::MotorType::kBrushless),
    m_motor3(kMotorID3, SparkMax::MotorType::kBrushless),
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
          .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
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

void ShooterSubsystem::ShootNear(){
    m_targetRPM = kForwardSpeedNear;
    m_mode = ShooterMode::kNear; 
    m_PID1.SetReference(m_targetRPM, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID2.SetReference(m_targetRPM, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID3.SetReference(m_targetRPM, rev::spark::SparkMax::ControlType::kVelocity);
}

void ShooterSubsystem::ShootFar(){
    m_targetRPM = kForwardSpeedFar;
    m_mode = ShooterMode::kFar; 
    m_PID1.SetReference(m_targetRPM, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID2.SetReference(m_targetRPM, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID3.SetReference(m_targetRPM, rev::spark::SparkMax::ControlType::kVelocity);
}

void ShooterSubsystem::Stop() {
    m_mode = ShooterMode::kStopped;
    m_PID1.SetReference(kStopShooter, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID2.SetReference(kStopShooter, rev::spark::SparkMax::ControlType::kVelocity);
    m_PID3.SetReference(kStopShooter, rev::spark::SparkMax::ControlType::kVelocity);
}
// body: configure inversion, PID values, etc.
bool ShooterSubsystem::AtSetpoint() {
    return
        fabs(m_encoder1.GetVelocity() - m_targetRPM) < kToleranceRPM &&
        fabs(m_encoder2.GetVelocity() - m_targetRPM) < kToleranceRPM &&
        fabs(m_encoder3.GetVelocity() - m_targetRPM) < kToleranceRPM;
}

double ShooterSubsystem::getRPM() {
    return (m_encoder1.GetVelocity()
          + m_encoder2.GetVelocity() 
          + m_encoder3.GetVelocity()) / 3.0;
}