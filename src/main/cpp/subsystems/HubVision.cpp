#include "subsystems/HubVision.h"

#include <algorithm>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

HubVision::HubVision()
    : m_camera{"camera0"} {}

void HubVision::Periodic() {
  // PhotonVision recomienda consumir resultados nuevos de la cámara
  // desde PhotonCamera y procesar los targets detectados.
  auto results = m_camera.GetAllUnreadResults();

  if (results.empty()) {
    frc::SmartDashboard::PutBoolean("HubVision/HasTarget", false);
    return;
  }

  std::optional<HubMeasurement> bestMeasurement;

  for (const auto& result : results) {
    if (!result.HasTargets()) {
      continue;
    }

    // Buscar entre todos los targets uno que pertenezca al HUB.
    // Elegimos el de menor ambigüedad de pose.
    std::optional<photon::PhotonTrackedTarget> bestHubTarget;

    for (const auto& target : result.GetTargets()) {
      if (!target.GetFiducialId()) {
        continue;
      }

      int fid = target.GetFiducialId();
      if (!IsHubTag(fid)) {
        continue;
      }

      if (!bestHubTarget.has_value()) {
        bestHubTarget = target;
      } else {
        double currentAmbiguity = target.GetPoseAmbiguity();
        double bestAmbiguity = bestHubTarget->GetPoseAmbiguity();

        // Ambigüedad menor = mejor medición
        if (currentAmbiguity >= 0.0 &&
            (bestAmbiguity < 0.0 || currentAmbiguity < bestAmbiguity)) {
          bestHubTarget = target;
        }
      }
    }

    if (!bestHubTarget.has_value()) {
      continue;
    }

    auto measurement =
        BuildMeasurementFromTarget(bestHubTarget.value(), result.GetTimestamp());
    if (measurement.has_value()) {
      bestMeasurement = measurement;
    }
  }

  if (bestMeasurement.has_value()) {
    m_latestMeasurement = bestMeasurement;

    frc::SmartDashboard::PutBoolean("HubVision/HasTarget", true);
    frc::SmartDashboard::PutNumber(
        "HubVision/TagId", m_latestMeasurement->tagId);
    frc::SmartDashboard::PutNumber(
        "HubVision/Forward_m",
        m_latestMeasurement->hubPoseRobotRelative3d.X().value());
    frc::SmartDashboard::PutNumber(
        "HubVision/Left_m",
        m_latestMeasurement->hubPoseRobotRelative3d.Y().value());
    frc::SmartDashboard::PutNumber(
        "HubVision/Distance_m", m_latestMeasurement->distance.value());
    frc::SmartDashboard::PutNumber(
        "HubVision/Bearing_deg", m_latestMeasurement->bearing.Degrees().value());
  } else {
    frc::SmartDashboard::PutBoolean("HubVision/HasTarget", false);
  }
}

bool HubVision::HasTarget() const {
  return m_latestMeasurement.has_value();
}

std::optional<HubVision::HubMeasurement>
HubVision::GetLatestHubMeasurement() const {
  return m_latestMeasurement;
}

units::meter_t HubVision::GetDistanceToHub() const {
  if (!m_latestMeasurement.has_value()) {
    return 0_m;
  }
  return m_latestMeasurement->distance;
}

frc::Rotation2d HubVision::GetBearingToHub() const {
  if (!m_latestMeasurement.has_value()) {
    return frc::Rotation2d{};
  }
  return m_latestMeasurement->bearing;
}

units::meter_t HubVision::GetForwardToHub() const {
  if (!m_latestMeasurement.has_value()) {
    return 0_m;
  }
  return m_latestMeasurement->hubPoseRobotRelative3d.X();
}

units::meter_t HubVision::GetLeftToHub() const {
  if (!m_latestMeasurement.has_value()) {
    return 0_m;
  }
  return m_latestMeasurement->hubPoseRobotRelative3d.Y();
}

bool HubVision::IsHubTag(int fiducialId) const {
  return std::find(kHubTagIds.begin(),
                   kHubTagIds.end(),
                   fiducialId) != kHubTagIds.end();
}

frc::Transform3d HubVision::GetTagToHub(int fiducialId) const {
  // IMPORTANTE:
  // Aquí debes poner la geometría real de cada tag respecto al centro del HUB.
  // Este ejemplo asume 4 tags, cada uno montado en lados distintos.
  //
  // Ajusta signos y rotaciones según el CAD real.
  //
  // Convención:
  // Transform desde el sistema del tag hacia el centro del HUB.

  switch (fiducialId) {
    case 10:
      return frc::Transform3d{
          frc::Translation3d{kHubCenterOffset, 0_m, 0_m},
          frc::Rotation3d{}};

    case 9:
      return frc::Transform3d{
          frc::Translation3d{kHubCenterOffset, 0.36_m, 0_m},
          frc::Rotation3d{}};

    // case 11:
    //   return frc::Transform3d{
    //       frc::Translation3d{kHubCenterOffset, 0.36_m, 0_m},
    //       frc::Rotation3d{0_rad, 0_rad, units::degree_t{180.0}}};

    default:
      return frc::Transform3d{};
  }
}

std::optional<HubVision::HubMeasurement>
HubVision::BuildMeasurementFromTarget(
    const photon::PhotonTrackedTarget& target,
    units::second_t timestamp) const {
  if (!target.GetFiducialId()) {
    return std::nullopt;
  }

  int fid = target.GetFiducialId();
  if (!IsHubTag(fid)) {
    return std::nullopt;
  }

  // PhotonVision entrega cámara -> target
  frc::Transform3d cameraToTag = target.GetBestCameraToTarget();

  // Transform conocida target/tag -> centro del HUB
  frc::Transform3d tagToHub = GetTagToHub(fid);

  // robot -> hub = robot -> camera -> tag -> hub
  frc::Pose3d robotOrigin{};
  frc::Pose3d hubPose =
      robotOrigin.TransformBy(kRobotToCam)
          .TransformBy(cameraToTag)
          .TransformBy(tagToHub);

  frc::Pose2d hubPose2d = hubPose.ToPose2d();

  units::meter_t forward = hubPose.X();
  units::meter_t left = hubPose.Y();

  units::meter_t distance =
      units::meter_t{std::hypot(forward.value(), left.value())};

  frc::Rotation2d bearing{double(forward), double(left)};

  return HubMeasurement{
      .hubPoseRobotRelative3d = hubPose,
      .hubPoseRobotRelative2d = hubPose2d,
      .distance = distance,
      .bearing = bearing,
      .timestamp = timestamp,
      .tagId = fid};
}