#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void SetRPM(double rpm);
  void Stop();
  bool AtSetpoint();

 private:
  rev::CANSparkMax m_motor1;
  rev::CANSparkMax m_motor2;
  rev::CANSparkMax m_motor3;

  rev::SparkMaxRelativeEncoder m_encoder1;
  rev::SparkMaxRelativeEncoder m_encoder2;
  rev::SparkMaxRelativeEncoder m_encoder3;

  rev::SparkMaxPIDController m_pid1;
  rev::SparkMaxPIDController m_pid2;
  rev::SparkMaxPIDController m_pid3;

  double m_targetRPM = 0.0;
};
