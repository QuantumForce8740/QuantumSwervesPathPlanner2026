#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"

class ShootNearCommand
: public frc2::CommandHelper<frc2::Command, ShootNearCommand> {
 public:
  explicit ShootNearCommand(ShooterSubsystem* shooter);

  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;

};
