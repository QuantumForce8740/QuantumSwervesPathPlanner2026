#include "commands/Intake/OscillateIntake.h"
#include <cmath>

OscillateIntake::OscillateIntake(Intake* intake)
    : m_intake(intake)
{
    AddRequirements({intake});
}

void OscillateIntake::Initialize()
{
    m_targetUp = true;
    m_intake->SetAngleDegrees(kAngleUp);
}

void OscillateIntake::Execute()
{
    if (m_intake->AtSetpoint())
    {
        m_targetUp = !m_targetUp;

        if(m_targetUp) 
        { 
            m_intake->SetAngleDegrees(kAngleUp); 
        } else
        {
            m_intake->SetAngleDegrees(kAngleDown);
        }
    }
}

void OscillateIntake::End(bool interrupted)
{
    m_intake->SetAngleDegrees(kAngleDown);
}

bool OscillateIntake::IsFinished()
{
    return false;
}