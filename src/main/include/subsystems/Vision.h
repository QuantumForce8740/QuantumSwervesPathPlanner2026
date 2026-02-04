#pragma once

#include <photon/PhotonCamera.h>
#include <frc2/command/SubsystemBase.h>

class Vision : public frc2::SubsystemBase {
    public:
        Vision();

        void Periodic() override;
        // std::vector<photon::PhotonPipelineResult> GetLatestResult() const { return m_latest; }
        // bool HasTargets() const { return m_latest.back().HasTargets(); }
    
    private: 
        photon::PhotonCamera m_camera{"Microsoft_LifeCam_HD-3000"};
        std::vector<photon::PhotonPipelineResult> m_latest;
        
};