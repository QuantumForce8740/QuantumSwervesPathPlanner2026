// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

RobotContainer::RobotContainer()
{
    pathplanner::NamedCommands::registerCommand("AlignToTarget", AlignToTarget(&drivetrain, &vision).ToPtr().WithTimeout(2_s));
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("RedCenter");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);
    alignPID.SetTolerance(1.5);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // drivetrain.SetDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     drivetrain.ApplyRequest([this]() -> auto&& {
    //         return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    //             .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
    //             .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
    //     })
    // );
    
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {

            double slowMultiplier = 1.0;

            if (joystick.LeftTrigger().Get() || joystick.LeftBumper().Get())
            {
                slowMultiplier = 0.2;
            }

            units::velocity::meters_per_second_t x = -joystick.GetLeftY() * MaxSpeed * slowMultiplier;
            units::velocity::meters_per_second_t y = -joystick.GetLeftX() * MaxSpeed * slowMultiplier;
            units::angular_velocity::radians_per_second_t rot = -joystick.GetRightX() * MaxAngularRate;

            bool alignPressed = joystick.RightBumper().Get() || joystick.RightTrigger().Get();

            if (alignPressed && vision.HasTarget())
            // if (alignPressed)
            {
                auto yawOpt = vision.GetTargetYaw();
                auto distanceOpt = vision.GetDistanceToTarget();

                if (yawOpt.has_value())
                {
                    double yaw = yawOpt.value();
                    double distance = distanceOpt.value();

                    // 1. Rotación automática
                    double pidOutput = alignPID.Calculate(yaw, 0);
                    pidOutput = std::clamp(pidOutput, -1.0, 1.0);

                    rot = units::radians_per_second_t{
                        pidOutput * MaxAngularRate.value()
                    };

                    // // Obtener velocidad y angulo dependiendo de la distancia
                    // auto params = ShooterLookupTable::GetShot(distance);

                    // // 2. Ajustar shooter automáticamente
                    // shooter.SetRPM(params.rpm);

                    // For testing
                    shooter.SetRPM(3000);

                    // // 3. Ajustar hood automáticamente
                    // hood.SetAngleDegrees(params.hoodAngle);

                    // For testing
                    hood.SetAngleDegrees(25.0);
                }
            }
            else
            {
                alignPID.Reset();
                shooter.SetRPM(1000);
                hood.SetAngleDegrees(0.0);
            }

            return drive.WithVelocityX(x)
                .WithVelocityY(y)
                .WithRotationalRate(rot);
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    // reset the field-centric heading on left bumper press
    joystick.Y().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    // Shoot
    joystick1.RightTrigger().WhileTrue(
    frc2::StartEndCommand(
        [&] { transfer.RunForward(); },  // Cuando presionas
        [&] { transfer.Stop(); },        // Cuando sueltas
        {&transfer}
        ).ToPtr()
    );

    joystick1.LeftTrigger().WhileTrue(
    frc2::StartEndCommand(
        [&] { transfer.RunReverse(); },
        [&] { transfer.Stop(); },
        {&transfer}
        ).ToPtr()
    );

    // Shoot
    joystick1.RightBumper().WhileTrue(
    frc2::StartEndCommand(
        [&] { feeder.RunForward(); },   // Cuando presionas
        [&] { feeder.Stop(); },         // Cuando sueltas
        {&feeder}
        ).ToPtr()
    );

    joystick1.LeftBumper().WhileTrue(
    frc2::StartEndCommand(
        [&] { feeder.RunReverse(); },
        [&] { feeder.Stop(); },
        {&feeder}
        ).ToPtr()
    );
    
    // joystick.POVLeft().OnTrue(SetHoodAtAngle(&hood, 25.0).ToPtr());

    // joystick1.LeftStick().OnTrue(SetHoodAtAngle(&hood, 25.0).ToPtr());

    // joystick1.POVDown().OnTrue(SetHoodAtAngle(&hood, 0.0).ToPtr());

    // Bajar intake
    joystick1.X().OnTrue(
        frc2::cmd::RunOnce(
            [this] { intake.Deploy(); },
            {&intake}
        )
    );

    // Subir intake
    joystick1.Y().OnTrue(
        frc2::cmd::RunOnce(
            [this] { intake.Retract(); },
            {&intake}
        )
    );

    // Mover pelotas
    joystick1.RightStick().WhileTrue(
        OscillateIntake(&intake).ToPtr()
    );

    // Intake In (mientras se presiona)
    // Agarra
    joystick1.A().WhileTrue(
        frc2::cmd::Run(
            [this] { intake.IntakeIn(); },
            {&intake}
        )
    ).OnFalse(
        frc2::cmd::RunOnce(
            [this] { intake.StopRollers(); },
            {&intake}
        )
    );

    // Intake Out
    joystick1.B().WhileTrue(
        frc2::cmd::Run(
            [this] { intake.IntakeOut(); },
            {&intake}
        )
    ).OnFalse(
        frc2::cmd::RunOnce(
            [this] { intake.StopRollers(); },
            {&intake}
        )
    );

    joystick1.POVUp().WhileTrue(
        ShootCommand(&transfer, &feeder).ToPtr()
    );

    joystick1.LeftStick().OnTrue(
    frc2::cmd::RunOnce(
        [this] { shooter.SetRPM(4500); },
        {&shooter}
        )
    );

    joystick1.POVDown().OnTrue(
    frc2::cmd::RunOnce(
        [this] { shooter.Stop(); },
        {&shooter}
        )
    );
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}
