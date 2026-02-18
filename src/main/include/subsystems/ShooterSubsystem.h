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
  void Stop();
  bool AtSetpoint();
  void ResetEncoder(); 
  void SetRPM (double rpm); 
  double getRPM();
  
  

 private:

  double m_targetRPM = 0.0; 


  SparkMax m_motor1;
  SparkMax m_motor2;
  SparkMax m_motor3;
  

  SparkClosedLoopController m_PID1; 
  SparkClosedLoopController m_PID2; 
  SparkClosedLoopController m_PID3; 

  SparkRelativeEncoder m_encoder1; 
  SparkRelativeEncoder m_encoder2;
  SparkRelativeEncoder m_encoder3;



  //SparkMaxConfig globalConfig; 
  //SparkRelativeEncoder m_encoder1;
  //SparkRelativeEncoder m_encoder2;
  //SparkRelativeEncoder m_encoder3; 

 // EncoderConfig EncoderConfig; 

  //reset encoders 

  //double m_targetRPM = 0.0;
};


