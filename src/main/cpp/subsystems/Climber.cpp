//

#include <subsystems/Climber.h>
#include <rev/SparkMax.h>


using namespace rev::spark;

ClimberSubsystem::ClimberSubsystem()
    : m_motor1(8, SparkMax::MotorType::kBrushless),
      m_motor2(9, SparkMax::MotorType::kBrushless) {

  // Invertir si físicamente lo necesita
  m_motor2.SetInverted(true);

  // Límite de corriente (recomendado para climber)


}

void ClimberSubsystem::ClimbUp() {
  m_motor1.Set(kClimbSpeed);
  m_motor2.Set(kClimbSpeed);
}

void ClimberSubsystem::Stop() {
  m_motor1.Set(0.0);
  m_motor2.Set(0.0);
}