// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/HubVision.h"
#include "subsystems/CommandSwerveDrivetrain.h"

class AlignToTarget : public frc2::CommandHelper<frc2::Command, AlignToTarget> {
 public:
   AlignToTarget(subsystems::CommandSwerveDrivetrain* drivetrain, HubVision* hubVision);

   void Initialize() override;

   void Execute() override;

   void End(bool interrupted) override;

   bool IsFinished() override;

 private:
   HubVision* m_hubVision;
   subsystems::CommandSwerveDrivetrain* m_drivetrain;
   frc::PIDController m_turnPID{0.08, 0.0, 0.0008};
   units::radians_per_second_t MaxAngularRate = 0.75_tps;
};