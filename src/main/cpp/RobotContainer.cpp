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

            double slowMultiplier = joystick.GetLeftTriggerAxis() > 0.1 ? 0.4 : 1.0; // aqui dice que si el gatillo tiene una precion mayor de 0.1  "joystick.GetLeftTriggerAxis() > 0.1 ?"
                                                                                    //entonces que "slowMultiplier" tenga un valor de .4  " 0.4 "
                                                                                    // Si no se queda con el valor de 1  ": 1.0;"

            units::velocity::meters_per_second_t x = -joystick.GetLeftY() * MaxSpeed * slowMultiplier;
            units::velocity::meters_per_second_t y = -joystick.GetLeftX() * MaxSpeed * slowMultiplier;
            units::angular_velocity::radians_per_second_t rot = -joystick.GetRightX() * MaxAngularRate * slowMultiplier;

            bool alignPressed = joystick.RightBumper().Get();

            if (alignPressed && vision.HasTarget())
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

                    // 2. Ajustar shooter automáticamente
                    // shooter.ShootAuto(distance);

                    // 3. Ajustar hood automáticamente
                    // hood.SetAngleFromDistance(distance);
                }
            }
            else
            {
                alignPID.Reset();
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

    // joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    // }));

    // joystick.POVUp().WhileTrue(
    //     drivetrain.ApplyRequest([this]() -> auto&& {
    //         return forwardStraight.WithVelocityX(0.5_mps).WithVelocityY(0_mps);
    //     })
    // );
    // joystick.POVDown().WhileTrue(
    //     drivetrain.ApplyRequest([this]() -> auto&& {
    //         return forwardStraight.WithVelocityX(-0.5_mps).WithVelocityY(0_mps);
    //     })
    // );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    // (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    // (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    // (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    // joystick.RightBumper().WhileTrue(&alignToTarget);

    // joystick1.A().WhileTrue(
    // frc2::StartEndCommand(
    //     [&] { transfer.RunForward(); },  // Cuando presionas
    //     [&] { transfer.Stop(); },        // Cuando sueltas
    //     {&transfer}
    //     ).ToPtr()
    // );

    // joystick1.B().WhileTrue(
    // frc2::StartEndCommand(
    //     [&] { transfer.RunReverse(); },
    //     [&] { transfer.Stop(); },
    //     {&transfer}
    //     ).ToPtr()
    // );

    // joystick1.X().WhileTrue(
    // frc2::StartEndCommand(
    //     [&] { feeder.RunForward(); },   // Cuando presionas
    //     [&] { feeder.Stop(); },         // Cuando sueltas
    //     {&feeder}
    //     ).ToPtr()
    // );

    // joystick1.Y().WhileTrue(
    // frc2::StartEndCommand(
    //     [&] { feeder.RunReverse(); },
    //     [&] { feeder.Stop(); },
    //     {&feeder}
    //     ).ToPtr()
    // );
    
    // joystick1.LeftTrigger().OnTrue(SetHoodAtAngle(&hood, 90.0).ToPtr());

    // joystick1.RightTrigger().OnTrue(SetHoodAtAngle(&hood, 0.0).ToPtr());

    // // A → Deploy intake
    // joystick.POVUp().OnTrue(
    //     frc2::cmd::RunOnce(
    //         [this] { intake.Deploy(); },
    //         {&intake}
    //     )
    // );

    // // B → Retract intake
    // joystick.POVDown().OnTrue(
    //     frc2::cmd::RunOnce(
    //         [this] { intake.Retract(); },
    //         {&intake}
    //     )
    // );

    // // Right Trigger → Intake In (mientras se presiona)
    // joystick.POVRight().WhileTrue(
    //     frc2::cmd::Run(
    //         [this] { intake.IntakeIn(); },
    //         {&intake}
    //     )
    // ).OnFalse(
    //     frc2::cmd::RunOnce(
    //         [this] { intake.StopRollers(); },
    //         {&intake}
    //     )
    // );

    // // Left Trigger → Intake Out
    // joystick.POVLeft().WhileTrue(
    //     frc2::cmd::Run(
    //         [this] { intake.IntakeOut(); },
    //         {&intake}
    //     )
    // ).OnFalse(
    //     frc2::cmd::RunOnce(
    //         [this] { intake.StopRollers(); },
    //         {&intake}
    //     )
    // );

    // joystick.LeftBumper().WhileTrue(
    //     AimCommand(&vision, &shooter, &hood, &drivetrain)
    // );

    // joystick.RightBumper().WhileTrue(
    //     ShootCommand(&shooter, &transfer, &feeder)
    // );

    // joystick.X().OnTrue(
    // frc2::cmd::RunOnce(
    //     [this] { shooter.SetRPM(3000); },
    //     {&shooter}
    //     )
    // );
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}
