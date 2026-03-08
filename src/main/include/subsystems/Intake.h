#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/RelativeEncoder.h>

using namespace rev::spark;

class Intake : public frc2::SubsystemBase {
public:
    Intake();

    void Deploy();
    void Retract();
    void SetAngleDegrees(double degrees);
    double GetAngleDegrees() const;
    void SetDeployPositionRotations(double rotations);
    bool AtSetpoint();

    void IntakeIn();
    void IntakeOut();
    void StopRollers();

private:

    static constexpr int kRollerMotorID = 20; // Cambiar al verdadero
    static constexpr int kDeployMotorID = 27; // Cambiar al verdadero

    SparkMax m_rollerMotor;
    SparkMax m_deployMotor;

    SparkClosedLoopController m_deployClosedLoop;
    SparkRelativeEncoder m_deployEncoder;

    static constexpr double kGearRatio = 50.0;

    static constexpr double kDeployAngle = 85.0;   // grados
    static constexpr double kRetractAngle = 0.0;

    double m_targetAngle; 

    double DegreesToMotorRotations(double degrees) const;
    double MotorRotationsToDegrees(double rotations) const;
};