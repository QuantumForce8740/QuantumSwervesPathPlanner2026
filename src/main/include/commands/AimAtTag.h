#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "subsystems/SwerveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class AimAtTagCommand
  : public frc2::CommandHelper<frc2::CommandBase, AimAtTagCommand> {
 public:
  AimAtTagCommand(SwerveSubsystem*, VisionSubsystem*);

  void Execute() override;
  bool IsFinished() override;

 private:
  SwerveSubsystem* m_swerve;
  VisionSubsystem* m_vision;
};
