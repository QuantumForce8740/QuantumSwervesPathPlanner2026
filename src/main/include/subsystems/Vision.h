#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <photon/PhotonCamera.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <frc2/command/SubsystemBase.h>

class Vision : public frc2::SubsystemBase {
    public:
        Vision();

        void Periodic() override;

        std::optional<double> GetTargetYaw();
        std::optional<double> GetDistanceToTarget();

        bool HasTarget();
    
    private: 
        photon::PhotonCamera m_camera{"camera0"};
        std::optional<int> lockedTag;
};