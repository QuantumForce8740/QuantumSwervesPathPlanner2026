#include "commands/AimCommand.h"

AimCommand::AimCommand(
    Vision* vision,
    Shooter* shooter,
    Hood* hood)
    : m_vision(vision),
      m_shooter(shooter),
      m_hood(hood)
{
    AddRequirements({shooter, hood});
}

void AimCommand::Initialize() {}

void AimCommand::Execute()
{
    auto yawOpt = m_vision->GetTargetYaw();
    auto distanceOpt = m_vision->GetDistanceToTarget();

    if (!yawOpt.has_value() || !distanceOpt.has_value())
        return;

    double yaw = yawOpt.value();
    double distance = distanceOpt.value();

    // 2. Ajustar RPM
    m_shooter->ShootAuto(distance);

    // 3. Ajustar hood
    m_hood->SetAngleFromDistance(distance);
}

void AimCommand::End(bool interrupted)
{}