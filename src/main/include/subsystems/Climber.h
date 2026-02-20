//

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void ClimbUp();
  void Stop();

 private:
  SparkMax m_motor1;
  SparkMax m_motor2;

  static constexpr double kClimbSpeed = 0.8;
};