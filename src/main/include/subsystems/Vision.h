#pragma once

#include <photon/PhotonCamera.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <frc2/command/SubsystemBase.h>

class Vision : public frc2::SubsystemBase {
    public:
        Vision();

        void Periodic() override;

        std::optional<double> GetTargetYaw();

        bool HasTarget();
    
    private: 
        photon::PhotonCamera m_camera{"Microsoft_LifeCam_HD-3000"};
};