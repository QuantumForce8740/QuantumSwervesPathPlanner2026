#include "subsystems/Vision.h"
#include <frc/smartdashboard/SmartDashboard.h>

Vision::Vision() {}

void Vision::Periodic() {}

std::optional<double> Vision::GetTargetYaw() {
    auto result = m_camera.GetLatestResult();

    if (!result.HasTargets())
        return std::nullopt;

    auto target = result.GetBestTarget();

    if (target.GetPoseAmbiguity() > 0.2)
        return std::nullopt;

    return target.GetYaw(); // grados
}

bool Vision::HasTarget()
{
    return m_camera.GetLatestResult().HasTargets();
}