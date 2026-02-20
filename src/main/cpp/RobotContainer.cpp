// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "RobotContainer.h"
#include "subsystems/shooterSubsystem.h"
#include "commands/ShootNearCommand.h"
#include "commands/ShootFarCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <memory>

RobotContainer::RobotContainer()
{
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings(){

    joystick.A().ToggleOnTrue(ShootNearCommand(&m_shooter).ToPtr());
    joystick.B().ToggleOnTrue(ShootFarCommand(&m_shooter).ToPtr());
    //joystick.X().ToggleOnTrue(StopShootCommand(&m_shooter).ToPtr());
   
      

    drivetrain.SetDefaultCommand(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate);
        })
    );

    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& { return brake; })
    );

    joystick.B().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return point.WithModuleDirection(
                frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()}
            );
        })
    );

    joystick.POVUp().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(0.5_mps).WithVelocityY(0_mps);
        })
    );

    joystick.POVDown().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(-0.5_mps).WithVelocityY(0_mps);
        })
    );

    (joystick.Back() && joystick.Y())
        .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));

    (joystick.Back() && joystick.X())
        .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));

    (joystick.Start() && joystick.Y())
        .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));

    (joystick.Start() && joystick.X())
        .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    joystick.LeftBumper().OnTrue(
        drivetrain.RunOnce([this] {
            drivetrain.SeedFieldCentric();
        })
    );

    drivetrain.RegisterTelemetry(
        [this](auto const& state) {
            logger.Telemeterize(state);
        }
    );
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}
