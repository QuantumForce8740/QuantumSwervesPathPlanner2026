#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "subsystems/ShooterSubsystem.h"

class ShootCommand
  : public frc2::CommandHelper<frc2::Command, ShootCommand> {
 public:
  ShootCommand(ShooterSubsystem* shooter, double rpm);

  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;
  double m_rpm;
};
