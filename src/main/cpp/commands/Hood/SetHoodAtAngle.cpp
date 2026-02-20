#include "commands/Hood/SetHoodAtAngle.h"
#include <cmath>

SetHoodAtAngle::SetHoodAtAngle(Hood* hood, double angleDegrees)
    : m_hood(hood),
      m_targetAngle(angleDegrees)
{
    AddRequirements({hood});
}

void SetHoodAtAngle::Initialize()
{
    m_hood->SetAngleDegrees(m_targetAngle);
}

bool SetHoodAtAngle::IsFinished()
{
    // Tolerancia de 1 grado
    return std::abs(m_hood->GetPositionDegrees() - m_targetAngle) < 1.0;
}