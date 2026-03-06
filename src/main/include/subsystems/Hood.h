#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/RelativeEncoder.h>

using namespace rev::spark;

class Hood : public frc2::SubsystemBase {
public:
    Hood();

    void SetAngleRotations(double rotations);
    void SetAngleDegrees(double degrees);
    void Stop();

    double GetPositionRotations() const;
    double GetPositionDegrees() const;

    // Para usar con visión
    void SetAngleFromDistance(double distanceMeters);

private:
    static constexpr int kMotorID = 25; // Cambiar al verdadero

    // Conversión (ajustar según tu relación mecánica)
    static constexpr double kGearRatio = 50.0;          // cambiar si hay reducción
    static constexpr double kDegreesPerRotation = 360.0 / kGearRatio;

    SparkMax m_motor;
    SparkClosedLoopController m_closedLoop;
    SparkRelativeEncoder m_encoder;

    double DegreesToRotations(double degrees) const;
};