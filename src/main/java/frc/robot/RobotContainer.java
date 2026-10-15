// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LinearArm;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    //swerve and chasis 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AutoFactory autoFactory = new AutoFactory(
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        drivetrain);
    //subsystems
    public final LinearArm linear_arm = new LinearArm();
    public final Shooter shooter = new Shooter();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //Swerve
        SwerveBinding();
        //Intake
        IntakeBinding();

        
    }
        

        
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
        
            Commands.runOnce(()-> DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)),
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            Commands.waitSeconds(5),
            autoFactory.driveToPoint(
                new Pose2d(2, 2, Rotation2d.fromDegrees(40)),
                new Pose2d(6,6,Rotation2d.fromDegrees(260))
                ),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }


    private void SwerveBinding(){
        drivetrain.register();
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    }
        private void IntakeBinding() {
        joystick.a().onTrue(Commands.either(
            linear_arm.goToSetpoint(()-> LinearArm.Setpoint.in),
            linear_arm.goToSetpoint(()-> LinearArm.Setpoint.out),
            ()-> linear_arm.getPosition().isNear(LinearArm.Setpoint.out.target, 
            Degree.of(10))
        ));
    }
}
