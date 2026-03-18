#include "commands/HubAimCommand.h"

HubAimCommand::HubAimCommand(
    Shooter* shooter,
    Hood* hood)
    : m_shooter(shooter),
      m_hood(hood)
{
    AddRequirements({shooter, hood});
}

void HubAimCommand::Initialize() {}

void HubAimCommand::Execute()
{
    // 2. Ajustar RPM
    m_shooter->SetRPM(3500);

    // 3. Ajustar hood
    m_hood->SetAngleDegrees(20.0);
}

void HubAimCommand::End(bool interrupted)
{}