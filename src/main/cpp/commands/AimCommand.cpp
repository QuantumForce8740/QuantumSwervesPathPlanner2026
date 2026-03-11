#include "commands/AimCommand.h"

AimCommand::AimCommand(
    HubVision* hubVision,
    Shooter* shooter,
    Hood* hood)
    : m_hubVision(hubVision),
      m_shooter(shooter),
      m_hood(hood)
{
    AddRequirements({shooter, hood});
}

void AimCommand::Initialize() {}

void AimCommand::Execute()
{
    auto distance = m_hubVision->GetDistanceToHub();

    // Obtener velocidad y angulo dependiendo de la distancia
    auto params = ShooterLookupTable::GetShot(distance.value());

    // 2. Ajustar shooter automáticamente
    m_shooter->SetRPM(params.rpm);

    // 3. Ajustar hood automáticamente
    m_hood->SetAngleDegrees(params.hoodAngle);
}

void AimCommand::End(bool interrupted)
{}