// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LinearArm;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(MaxSpeed);
    //subsystems
    public final Subsystems subsystems = new Subsystems();
    public final AutoFactory autoFactory = new AutoFactory(
        new PIDController(2.3, 0, 0),
        new PIDController(2.3, 0, 0),
        subsystems.drivetrain);
    public FollowPath.Builder pathBuilder;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //Swerve
        SwerveBinding();
        //Intake
        IntakeBinding();
        //shoter
        ShooterBinding();
                
    }

    public void Periodic(){
        subsystems.vision.periodic();
    }
                
        

    public Command getAutonomousCommand() {
        return pathBuilder.build(new Path("Auto1"));
    }
    private Kicker.Setpoint kickerSetpoint = Kicker.Setpoint.stop;
    private Shooter.Setpoint shooterSetpoint = Shooter.Setpoint.Near;
    private Indexer.Setpoint indexerSetpoint = Indexer.Setpoint.stop;
    private void ShooterBinding() {
        Controllers.driveController.L1().onTrue(Commands.runOnce(()->{
            System.out.println("check");
            if (shooterSetpoint == Shooter.Setpoint.Near) {
                shooterSetpoint = Shooter.Setpoint.Far;
            }
            else{
                shooterSetpoint = Shooter.Setpoint.Near;
            }
        }));
        subsystems.shooter.setDefaultCommand(subsystems.shooter.drive(()->{return shooterSetpoint;}));
        subsystems.shooter.createIsNearTargetTrigger(RotationsPerSecond.of(2)).onTrue(Commands.runOnce(()->{
            indexerSetpoint = Indexer.Setpoint.index;
        })).onFalse(Commands.runOnce(()->{
            indexerSetpoint = Indexer.Setpoint.stop;
        }));
        subsystems.indexer.setDefaultCommand(subsystems.indexer.drive(()->{return indexerSetpoint;}));
        subsystems.indexer.createIsNearTargetTrigger(RotationsPerSecond.of(1)).onTrue(Commands.runOnce(()->{
            kickerSetpoint = Kicker.Setpoint.kicking;
        })).onFalse(Commands.runOnce(()->{
            kickerSetpoint = Kicker.Setpoint.stop;
        }));
        subsystems.kicker.setDefaultCommand(subsystems.kicker.drive(()->{return kickerSetpoint;}));
    }


    private void SwerveBinding(){
        subsystems.drivetrain.register();

        pathBuilder = new FollowPath.Builder(
            subsystems.drivetrain, ()-> subsystems.drivetrain.getState().Pose, 
            ()->subsystems.drivetrain.getState().Speeds, (speeds)->{
                subsystems.drivetrain.setControl(
                    driveRobotCentric.withVelocityX(speeds.vxMetersPerSecond + Controllers.driveController.getLeftY())
                    .withVelocityY(speeds.vyMetersPerSecond + Controllers.driveController.getLeftX())
                    .withRotationalRate(speeds.omegaRadiansPerSecond));
            }, 
            new PIDController(2.3, 0, 0), 
            new PIDController(2.3, 0, 0), 
            new PIDController(0.2, 0, 0))
            .withDefaultShouldFlip().withPoseReset(subsystems.drivetrain::resetPose)
            .withTRatioBasedTranslationHandoffs(true);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        subsystems.drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            subsystems.drivetrain.applyRequest(() ->
                drive.withVelocityX(-Controllers.driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Controllers.driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Controllers.driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            subsystems.drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // Reset the field-centric heading on left bumper press.
        Controllers.driveController.touchpad().onTrue(subsystems.drivetrain.runOnce(subsystems.drivetrain::seedFieldCentric));

        subsystems.drivetrain.registerTelemetry(logger::telemeterize);
    }
    private LinearArm.Setpoint LinearArmsetpoint = LinearArm.Setpoint.in;
    private void IntakeBinding() {
        // joystick.a().onTrue(Commands.runOnce(()->{
        //     if (LinearArmsetpoint == LinearArm.Setpoint.in) {
        //         LinearArmsetpoint = LinearArm.Setpoint.out;
        //     }
        //     else{
        //         LinearArmsetpoint = LinearArm.Setpoint.in;
        //     }
        // }));
    subsystems.linear_arm.setDefaultCommand(subsystems.linear_arm.goToSetpoint(()-> {return LinearArmsetpoint;}));
    new Trigger(()->subsystems.linear_arm.getPosition()
        .isNear(LinearArm.Setpoint.out.target, Degree.of(2)))
            .onTrue(subsystems.intakeRoller.drive(()->IntakeRoller.Setpoint.Run))
            .onFalse(subsystems.intakeRoller.drive(()->IntakeRoller.Setpoint.Stop));
    }
    public void resetRobot(){
        System.out.println(DriverStation.isDisabled());
        CommandScheduler.getInstance().schedule(
            Commands.parallel(            
            subsystems.linear_arm.calibrateZero(),
            subsystems.shooter.coast(),
            subsystems.intakeRoller.coast(),
            subsystems.drivetrain.applyRequest(()->new SwerveRequest.Idle()).withTimeout(1))
            .withTimeout(4)
            );
    }
}
