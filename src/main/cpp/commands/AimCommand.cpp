#include "commands/AimCommand.h"

AimCommand::AimCommand(
    Vision* vision,
    // ShooterSubsystem* shooter,
    Hood* hood,
    subsystems::CommandSwerveDrivetrain* drive)
    : m_vision(vision),
    //   m_shooter(shooter),
      m_hood(hood),
      m_drive(drive)
{
    // AddRequirements({shooter, hood, drive});
    AddRequirements({hood, drive});
    m_turnPID.SetTolerance(1.5); // grados
}

void AimCommand::Initialize()
{
    m_turnPID.Reset();
}

void AimCommand::Execute()
{
    auto yawOpt = m_vision->GetTargetYaw();
    auto distanceOpt = m_vision->GetDistanceToTarget();

    if (!yawOpt.has_value() || !distanceOpt.has_value())
        return;

    double yaw = yawOpt.value();
    double distance = distanceOpt.value();

    // 1. Alinear robot
    double rotSpeed = m_turnPID.Calculate(yaw, 0);

    // limitar velocidad angular
    rotSpeed = std::clamp(rotSpeed, -3.0, 3.0);

    frc::ChassisSpeeds speeds{
        0_mps,
        0_mps,
        units::radians_per_second_t{rotSpeed}
    };

    m_drive->DriveRobotRelative(speeds);

    // 2. Ajustar RPM
    // m_shooter->ShootAuto(distance);

    // 3. Ajustar hood
    m_hood->SetAngleFromDistance(distance);
}

void AimCommand::End(bool interrupted)
{
    m_drive->Stop();
}