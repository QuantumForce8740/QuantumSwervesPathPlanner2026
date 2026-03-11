// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Vision/AlignToTarget.h"
#include "subsystems/CommandSwerveDrivetrain.h"

#include <cmath>

AlignToTarget::AlignToTarget(subsystems::CommandSwerveDrivetrain* drivetrain, HubVision* hubVision) :
    m_drivetrain(drivetrain), m_hubVision(hubVision) {
        AddRequirements({drivetrain});
        m_turnPID.SetTolerance(1.5); // grados
}

void AlignToTarget::Initialize() {
    m_turnPID.Reset();
}

void AlignToTarget::Execute() {
    auto angle = m_hubVision->GetBearingToHub().Radians().value();

    double pidOutput = m_turnPID.Calculate(angle, 0);
    pidOutput = std::clamp(pidOutput, -1.0, 1.0);

    units::angular_velocity::radians_per_second_t rot = units::radians_per_second_t{
        angle * MaxAngularRate.value()
    };

    frc::ChassisSpeeds speeds{
        0_mps,
        0_mps,
        units::radians_per_second_t{rot}
    };

    m_drivetrain->DriveRobotRelative(speeds);
}

void AlignToTarget::End(bool interrupted) {
    m_drivetrain->Stop();
}

bool AlignToTarget::IsFinished() {
    return false;
}