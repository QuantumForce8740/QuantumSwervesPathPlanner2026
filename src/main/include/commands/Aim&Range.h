#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "subsystems/CommandSwerveSubsystem.h"
#include "subsystems/vision.h"

class AimAndRangeCommand
  : public frc2::CommandHelper<frc2::CommandBase, AimAndRangeCommand> {
 public:
  AimAndRangeCommand(SwerveSubsystem*, VisionSubsystem*);

  void Execute() override;

 private:
  SwerveSubsystem* m_swerve;
  VisionSubsystem* m_vision;

  const units::meter_t kTargetDistance = 1.0_m;
};
