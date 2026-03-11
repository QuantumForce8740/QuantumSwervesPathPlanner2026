#include "commands/NeutralAimCommand.h"

NeutralAimCommand::NeutralAimCommand(
    Shooter* shooter,
    Hood* hood)
    : m_shooter(shooter),
      m_hood(hood)
{
    AddRequirements({shooter, hood});
}

void NeutralAimCommand::Initialize() {}

void NeutralAimCommand::Execute()
{
    // 2. Ajustar RPM
    m_shooter->SetRPM(4000);

    // 3. Ajustar hood
    m_hood->SetAngleDegrees(25.0);
}

void NeutralAimCommand::End(bool interrupted)
{}