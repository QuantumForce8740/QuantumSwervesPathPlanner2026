#include "subsystems/Vision.h"
#include <cmath>

Vision::Vision() {
    frc::SmartDashboard::PutNumber("Has Target", HasTarget());
    frc::SmartDashboard::PutNumber("Yaw", GetTargetYaw().value_or(0.0));
    frc::SmartDashboard::PutNumber("Distance", GetDistanceToTarget().value_or(0.0));
}

void Vision::Periodic() {}

std::optional<double> Vision::GetTargetYaw() {
    auto result = m_camera.GetLatestResult();

    if (!result.HasTargets())
    {
        lockedTag.reset();
        return std::nullopt;
    }

    for (auto target : result.GetTargets())
    {
        if (!lockedTag.has_value())
        {
            lockedTag = target.GetFiducialId();
        }
        
        if (target.GetFiducialId() == lockedTag.value())
        {
            return target.GetYaw();
        }
        else
        {
            return std::nullopt;
        }
    }

    // auto target = result.GetBestTarget();

    // if (target.GetPoseAmbiguity() > 0.2)
    //     return std::nullopt;

    // return target.GetYaw(); // grados
}

std::optional<double> Vision::GetDistanceToTarget()
{
    auto result = m_camera.GetLatestResult();

    if (!result.HasTargets())
        return std::nullopt;

    auto target = result.GetBestTarget();

    if (target.GetPoseAmbiguity() > 0.2)
        return std::nullopt;

    auto transform = target.GetBestCameraToTarget();

    double x = transform.X().value();  // metros
    double y = transform.Y().value();  // metros

    // Distancia horizontal real
    double distance = std::sqrt(x * x + y * y);

    return distance;
}

bool Vision::HasTarget()
{
    return m_camera.GetLatestResult().HasTargets();
}