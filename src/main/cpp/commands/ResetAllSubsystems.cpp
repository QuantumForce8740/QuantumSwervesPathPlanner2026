#include "commands/ResetAllSubsystems.h"

ResetAllSubsystems::ResetAllSubsystems(
    Shooter* shooter,
    Hood* hood,
    Transfer* transfer, 
    Feeder* feeder)
    : m_shooter(shooter),
      m_hood(hood),
      m_transfer(transfer),
      m_feeder(feeder)
{
    AddRequirements({shooter, hood, transfer, feeder});
}

void ResetAllSubsystems::Initialize() {}

void ResetAllSubsystems::Execute()
{
    // 1. Ajustar RPM
    m_shooter->SetRPM(0.0);

    // 2. Ajustar hood
    m_hood->SetAngleDegrees(0.0);

    m_transfer->Stop();

    m_feeder->Stop();
}

void ResetAllSubsystems::End(bool interrupted)
{}