

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <units/velocity.h>

#include "subsystems/CommandSwerveDraintrain.h"
#include "subsystems/vision.h"

class GetInRangeCommand
  : public frc2::CommandHelper<frc2::CommandBase, GetInRangeCommand> {
 public:
  GetInRangeCommand(SwerveSubsystem*, VisionSubsystem*);

  void Execute() override;
  bool IsFinished() override;

 private:
  SwerveSubsystem* m_swerve;
  VisionSubsystem* m_vision;

  const units::meter_t kTargetDistance = 1.0_m;
};
