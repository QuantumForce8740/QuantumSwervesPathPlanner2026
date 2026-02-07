// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Vision/AlignToTarget.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <cmath>

AlignToTarget::AlignToTarget(subsystems::CommandSwerveDrivetrain* drivetrain, Vision* vision) :
    m_drivetrain(drivetrain), m_vision(vision) {
        AddRequirements({drivetrain});
        m_turnPID.SetTolerance(1.5); // grados
}

void AlignToTarget::Initialize() {
    m_turnPID.Reset();
}

void AlignToTarget::Execute() {
    auto yawOpt = m_vision->GetTargetYaw();

    if (!yawOpt.has_value()) {
        m_drivetrain->Stop();
        return;
    }

    double yaw = yawOpt.value();

    double rotSpeed = m_turnPID.Calculate(yaw, 0);

    // limitar velocidad angular
    rotSpeed = std::clamp(rotSpeed, -3.0, 3.0);

    frc::ChassisSpeeds speeds{
        0_mps,
        0_mps,
        units::radians_per_second_t{rotSpeed}
    };

    m_drivetrain->DriveRobotRelative(speeds);
}

void AlignToTarget::End(bool interrupted) {
    m_drivetrain->Stop();
}

bool AlignToTarget::IsFinished() {
    return false;
}