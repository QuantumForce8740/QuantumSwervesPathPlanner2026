#include "subsystems/vision.h"

VisionSubsystem::VisionSubsystem() {}

bool VisionSubsystem::HasTarget() {
  return m_camera.GetLatestResult().HasTargets();
}

double VisionSubsystem::GetYaw() {
  auto result = m_camera.GetLatestResult();
  if (!result.HasTargets()) return 0.0;
  return result.GetBestTarget().GetYaw();
}

units::meter_t VisionSubsystem::GetDistance() {
  auto result = m_camera.GetLatestResult();
  if (!result.HasTargets()) return 0_m;

  return result.GetBestTarget()
      .GetBestCameraToTarget()
      .Translation()
      .Norm();
}
