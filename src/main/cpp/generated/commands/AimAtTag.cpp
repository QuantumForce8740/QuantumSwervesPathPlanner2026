#include "commands/AimAtTagCommand.h"
#include <algorithm>
#include <cmath>

AimAtTagCommand::AimAtTagCommand(
    SwerveSubsystem* swerve, VisionSubsystem* vision)
  : m_swerve(swerve), m_vision(vision) {
  AddRequirements(swerve);
}

void AimAtTagCommand::Execute() {
  if (!m_vision->HasTarget()) {
    m_swerve->Drive(0_mps, 0_mps, 0_rad_per_s, true);
    return;
  }

  double yaw = m_vision->GetYaw(); // grados

  auto omega = units::radians_per_second_t(
      std::clamp(yaw * 0.03, -2.0, 2.0));

  m_swerve->Drive(0_mps, 0_mps, omega, true);
}

bool AimAtTagCommand::IsFinished() {
  return std::abs(m_vision->GetYaw()) < 2.0;
}


bool AimAtTagCommand::IsFinished() {
  return std::abs(m_vision->GetYaw()) < 2.0;
}
