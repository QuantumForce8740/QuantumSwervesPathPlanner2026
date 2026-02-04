// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Vision/AimAtTarget.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <cmath>

AimAtTarget::AimAtTarget(){
}

void AimAtTarget::Initialize() {
    targetVisible = false;
    targetYaw = 0.0;
}

void AimAtTarget::Execute() {
    auto results = camera.GetAllUnreadResults();

    if (results.size() > 0) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        auto result = results[results.size() - 1];
        if (result.HasTargets()) {
            // At least one AprilTag was seen by the camera
            for (auto& target : result.GetTargets()) {
                if (target.GetFiducialId() == 12) {
                    // Found Tag 7, record its information
                    targetYaw = target.GetYaw();
                    targetVisible = true;
                }
            }
        }
    }

    turn = -1.0 * targetYaw * VISION_TURN_kP;
}

double AimAtTarget::GetTurn(){
    return turn;
}

std::vector<photon::PhotonPipelineResult> AimAtTarget::GetResults(){
    return camera.GetAllUnreadResults();
}

void AimAtTarget::End(bool interrupted) {
}

bool AimAtTarget::IsFinished() {
}