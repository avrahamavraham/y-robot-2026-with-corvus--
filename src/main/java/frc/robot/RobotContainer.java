// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Commands.SwerveCommands;
import frc.robot.autos.AutoFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LinearArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
        new PIDController(2.3, 0, 0),
        new PIDController(2.3, 0, 0),
        drivetrain);
    //subsystems
    public final LinearArm linear_arm = new LinearArm();
    public final Shooter shooter = new Shooter();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public final Indexer indexer = new Indexer();
    public final Kicker kicker = new Kicker();
    public final  Vision vision;
    public RobotContainer() {
        if (Robot.isReal()) {
            vision = new Vision(drivetrain, 
            new VisionIOLimelight("limelight-gg", ()->drivetrain.getState().Pose.getRotation(), 0.2));
        } else {
            vision = new Vision(drivetrain,
            new VisionIOPhotonVisionSim("limelight-gg", 
                new Transform3d(-0.1808, -0.023073, 0.44788, new Rotation3d(0,0,Math.PI)), 
                ()->drivetrain.getState().Pose, 0.2)            
            );
        }
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
        vision.periodic();
    }
                
        

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
        
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            Commands.waitSeconds(1),
            autoFactory.driveToPoint(
                new Pose2d(2, 2, Rotation2d.fromDegrees(40)),
                new Pose2d(6,6,Rotation2d.fromDegrees(260)),
                new Pose2d()
                ),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    private Kicker.Setpoint kickerSetpoint = Kicker.Setpoint.stop;
    private Shooter.Setpoint shooterSetpoint = Shooter.Setpoint.Near;
    private Indexer.Setpoint indexerSetpoint = Indexer.Setpoint.stop;
    private void ShooterBinding() {
        joystick.b().onTrue(Commands.runOnce(()->{
            System.out.println("check");
            if (shooterSetpoint == Shooter.Setpoint.Near) {
                shooterSetpoint = Shooter.Setpoint.Far;
            }
            else{
                shooterSetpoint = Shooter.Setpoint.Near;
            }
        }));
        shooter.setDefaultCommand(shooter.drive(()->{return shooterSetpoint;}));
        shooter.createIsNearTargetTrigger(RotationsPerSecond.of(2)).onTrue(Commands.runOnce(()->{
            indexerSetpoint = Indexer.Setpoint.index;
        })).onFalse(Commands.runOnce(()->{
            indexerSetpoint = Indexer.Setpoint.stop;
        }));
        indexer.setDefaultCommand(indexer.drive(()->{return indexerSetpoint;}));
        indexer.createIsNearTargetTrigger(RotationsPerSecond.of(1)).onTrue(Commands.runOnce(()->{
            kickerSetpoint = Kicker.Setpoint.kicking;
        })).onFalse(Commands.runOnce(()->{
            kickerSetpoint = Kicker.Setpoint.stop;
        }));
        kicker.setDefaultCommand(kicker.drive(()->{return kickerSetpoint;}));
    }


    private void SwerveBinding(){
        SwerveCommands swerveCommands = new SwerveCommands(drivetrain);
        drivetrain.register();
        //pathPlanner
        // PathPlannerOptional.configureAutoBuilder(drivetrain);
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
        joystick.a().whileTrue(
            swerveCommands.FollowHub(
                ()->-joystick.getLeftY() * MaxSpeed,()-> -joystick.getLeftX() * MaxSpeed));
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
    linear_arm.setDefaultCommand(linear_arm.goToSetpoint(()-> {return LinearArmsetpoint;}));
    }
    public void resetRobot(){
        System.out.println(DriverStation.isDisabled());
        CommandScheduler.getInstance().schedule(
            linear_arm.calibrateZero(),
            shooter.coast(),
            intakeRoller.coast(),
            drivetrain.applyRequest(()->new SwerveRequest.Idle()).withTimeout(1)
            );
    }
}
