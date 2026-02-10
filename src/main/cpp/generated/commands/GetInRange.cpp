#include "commands/GetInRange.h"
#include <algorithm>
#include <cmath>

GetInRangeCommand::GetInRangeCommand(
    SwerveSubsystem* swerve, VisionSubsystem* vision)
  : m_swerve(swerve), m_vision(vision) {
  AddRequirements(swerve);
}

void GetInRangeCommand::Execute() {
  if (!m_vision->HasTarget()) {
    m_swerve->Drive(0_mps, 0_mps, 0_rad_per_s, true);
    return;
  }

  units::meter_t error =
      m_vision->GetDistance() - kTargetDistance;

  auto vx = units::meters_per_second_t(
      std::clamp(error.value() * 1.2, -1.0, 1.0));

  m_swerve->Drive(vx, 0_mps, 0_rad_per_s, true);
}

bool GetInRangeCommand::IsFinished() {
  return std::abs(
      (m_vision->GetDistance() - kTargetDistance).value()) < 0.1;
}
