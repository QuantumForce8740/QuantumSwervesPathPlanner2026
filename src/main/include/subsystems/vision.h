#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <units/length.h>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  bool HasTarget();
  double GetYaw();                 // degrees
  units::meter_t GetDistance();    // meters

 private:
  photon::PhotonCamera m_cameraVision{"photonvision"};
};
