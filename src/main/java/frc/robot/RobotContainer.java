// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.TunerConstants;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired tp speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick2 = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Subsystem Initialization
        private final ShooterSubsystem shooterSS =  new ShooterSubsystem(true, true, drivetrain);
        private final HopperSubsystem hopperSS = new HopperSubsystem(false);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick2.getLeftY() * MaxSpeed * 0.15) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick2.getLeftX() * MaxSpeed * 0.2) // Drive left with negative X (left)
                    .withRotationalRate(-joystick2.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick2.rightBumper().onTrue(
        //     Commands.sequence(
        //         shooterSS.setAngleAndVelocityCommand(25, 5), hopperSS.testCommand(true)
        //     )
        // ).onFalse(
        //     Commands.sequence(
        //         shooterSS.setAngleAndVelocityCommand(0, 0), hopperSS.testCommand(false)
        //     )
        // );
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)   
        );

        joystick2.start().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick2.a().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick2.getLeftY(), -joystick2.getLeftX()))
        ));

        // Reset the field-centric heading on left bumper press.
        joystick2.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}