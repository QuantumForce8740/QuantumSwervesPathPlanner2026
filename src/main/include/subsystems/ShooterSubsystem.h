#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/RelativeEncoder.h>
#include <frc/Encoder.h>

using namespace rev::spark;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void Periodic() override;
  bool AtSetpoint();
  void ResetEncoder(); 
  double getRPM();
  void Stop();
  void ShootNear(); 
  void ShootFar(); 

  
  //para q shooter y feeder esten sincroniados, se puede usar el mismo modo para ambos y asi no hay que estar seteando cada vez el RPM, sino solo el modo.
  enum class ShooterMode {
    kStopped,
    kNear, 
    kFar
  }; 

public: 
ShooterMode GetMode() const { return m_mode;} 

 private:

 ShooterMode m_mode = ShooterMode::kStopped;
 double m_targetRPM = 0.0;

 static constexpr int kMotorID =20; 
 static constexpr int kMotorID2 = 21;
 static constexpr int kMotorID3 = 24;

 static constexpr double kForwardSpeedNear = 3000.0;
 static constexpr double kForwardSpeedFar = 4600.0; 
 static constexpr double kStopShooter = 0.0; 
 static constexpr double kToleranceRPM = 100.0;

  

  SparkMax m_motor1;
  SparkMax m_motor2;
  SparkMax m_motor3;
  

  SparkClosedLoopController m_PID1; 
  SparkClosedLoopController m_PID2; 
  SparkClosedLoopController m_PID3; 

  SparkRelativeEncoder m_encoder1; 
  SparkRelativeEncoder m_encoder2;
  SparkRelativeEncoder m_encoder3;

};


