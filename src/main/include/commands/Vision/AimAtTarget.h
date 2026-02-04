// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <photon/PhotonCamera.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/CommandSwerveDrivetrain.h"

class AimAtTarget : public frc2::CommandHelper<frc2::Command, AimAtTarget> {
 public:
  /**
   * Creates a new AimAtTarget.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  AimAtTarget();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  double GetTurn();

  std::vector<photon::PhotonPipelineResult> GetResults();

 private:
    photon::PhotonCamera camera{"Microsoft_LifeCam_HD-3000"};
    static constexpr double VISION_TURN_kP = 0.01;
    bool targetVisible = false;
    double targetYaw = 0.0;
    double turn = 0.0;
};