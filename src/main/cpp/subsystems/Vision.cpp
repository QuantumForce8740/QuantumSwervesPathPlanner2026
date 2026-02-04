#include "subsystems/Vision.h"
#include <frc/smartdashboard/SmartDashboard.h>

Vision::Vision() {}

void Vision::Periodic(){
    m_latest = m_camera.GetAllUnreadResults();

    frc::SmartDashboard::PutBoolean("Has targets", m_latest.back().HasTargets());
    frc::SmartDashboard::PutNumber("Unread Results", m_latest.size());

    if(m_latest.back().HasTargets()){
        auto target = m_latest.back().GetBestTarget();
        frc::SmartDashboard::PutNumber("Target Yaw", target.GetYaw());
    }
}