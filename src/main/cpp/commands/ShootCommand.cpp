#include "commands/ShootCommand.h"

ShootCommand::ShootCommand(
    // Shooter* shooter,
    Transfer* transfer,
    Feeder* feeder)
    : 
    //   m_shooter(shooter),
      m_transfer(transfer),
      m_feeder(feeder)
{
    AddRequirements({transfer, feeder});
}

void ShootCommand::Execute()
{
    // if (m_shooter->AtSetpoint())
    // {
    //     m_transfer->RunForward();
    //     m_feeder->RunForward();
    // }
    m_transfer->RunForward();
    m_feeder->RunForward();
}

void ShootCommand::End(bool interrupted)
{
    m_transfer->Stop();
    m_feeder->Stop();
}