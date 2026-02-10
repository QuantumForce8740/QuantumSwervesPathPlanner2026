#include "subsystems/Shooter.h"

ShooterSubsystem::ShooterSubsystem()
  : m_motor1(1, rev::CANSparkMax::MotorType::kBrushless),
    m_motor2(2, rev::CANSparkMax::MotorType::kBrushless),
    m_motor3(3, rev::CANSparkMax::MotorType::kBrushless),
    m_encoder1(m_motor1.GetEncoder()),
    m_encoder2(m_motor2.GetEncoder()),
    m_encoder3(m_motor3.GetEncoder()),
    m_pid1(m_motor1.GetPIDController()),
    m_pid2(m_motor2.GetPIDController()),
    m_pid3(m_motor3.GetPIDController()) {

  //  Reset & safety
  m_motor1.RestoreFactoryDefaults();
  m_motor2.RestoreFactoryDefaults();
  m_motor3.RestoreFactoryDefaults();

  // Invertir si es necesario
  m_motor1.SetInverted(false);
  m_motor2.SetInverted(false);
  m_motor3.SetInverted(false);

  // PID values (P inicial recomendado)
  double kP = 0.0002;
  double kI = 0.0;
  double kD = 0.0;
  double kFF = 0.00018;

  for (auto pid : {&m_pid1, &m_pid2, &m_pid3}) {
    pid->SetP(kP);
    pid->SetI(kI);
    pid->SetD(kD);
    pid->SetFF(kFF);
    pid->SetOutputRange(-1.0, 1.0);
  }
}

void ShooterSubsystem::SetRPM(double rpm) {
  m_targetRPM = rpm;

  m_pid1.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
  m_pid2.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
  m_pid3.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
}

void ShooterSubsystem::Stop() {
  SetRPM(0);
}

bool ShooterSubsystem::AtSetpoint() {
  return
    fabs(m_encoder1.GetVelocity() - m_targetRPM) < 100 &&
    fabs(m_encoder2.GetVelocity() - m_targetRPM) < 100 &&
    fabs(m_encoder3.GetVelocity() - m_targetRPM) < 100;
}
