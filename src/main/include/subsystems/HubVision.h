#pragma once

#include <optional>
#include <unordered_map>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

class HubVision : public frc2::SubsystemBase {
 public:
  struct HubMeasurement {
    frc::Pose3d hubPoseRobotRelative3d;
    frc::Pose2d hubPoseRobotRelative2d;
    units::meter_t distance;
    frc::Rotation2d bearing;
    units::second_t timestamp;
    int tagId;
  };

    HubVision();

    void Periodic() override;

    bool HasTarget() const;
    std::optional<HubMeasurement> GetLatestHubMeasurement() const;

    units::meter_t GetDistanceToHub() const;
    frc::Rotation2d GetBearingToHub() const;
    units::meter_t GetForwardToHub() const;
    units::meter_t GetLeftToHub() const;

 private:
    photon::PhotonCamera m_camera;

    std::optional<HubMeasurement> m_latestMeasurement;

    bool IsHubTag(int fiducialId) const;
    frc::Transform3d GetTagToHub(int fiducialId) const;
    std::optional<HubMeasurement> BuildMeasurementFromTarget(
        const photon::PhotonTrackedTarget& target,
        units::second_t timestamp) const;

    // Robot -> Camera
    // Coordenadas WPILib:
    // +X forward, +Y left, +Z up
    const frc::Transform3d kRobotToCam{
        frc::Translation3d{-0.05_m, 0.00_m, 0.53_m},
        frc::Rotation3d{0_rad, units::degree_t{13.0}, 0_rad}
    };

    // IDs de AprilTags que pertenecen al HUB
    const std::array<int, 4> kHubTagIds{9, 10, 25, 26};

    // Ejemplo: distancia desde la cara del tag hasta el centro del HUB.
    // CAMBIA esto por tu geometría real medida en CAD.
    const units::meter_t kHubCenterOffset = -0.56_m;

};