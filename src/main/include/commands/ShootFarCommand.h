#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"

class ShootFarCommand
: public frc2::CommandHelper<frc2::Command, ShootFarCommand> {
 public:
  explicit ShootFarCommand(ShooterSubsystem* shooter);

  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;

};
