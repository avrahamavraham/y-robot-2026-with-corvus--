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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Buttons.DriverButtons;
import frc.robot.Buttons.OperatorButtons;
import frc.robot.Commands.IntakeCommands;
import frc.robot.Commands.ScoringCommands;
import frc.robot.Commands.SwerveCommands;
import frc.robot.ShootOnTheMove.SOTM;
import frc.robot.autos.Auto1Starting;
import frc.robot.autos.AutoFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.Ps5ControllerSim;
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

    //commands (ported from robot2026Catalyst)
    public final SwerveCommands swerveCommands = new SwerveCommands(drivetrain);
    /** Builds the shot lookup table here, at startup, never during a match. */
    public final SOTM sotm = new SOTM(drivetrain);
    public final IntakeCommands intakeCommands = new IntakeCommands(linear_arm, intakeRoller);
    public final ScoringCommands scoringCommands =
        new ScoringCommands(shooter, kicker, indexer, drivetrain, swerveCommands, sotm);
    private final Auto1Starting auto1Starting = new Auto1Starting(autoFactory, drivetrain);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    //simulated PS5 controllers (only do anything when ticked in the dashboard, see Ps5ControllerSim)
    private final Ps5ControllerSim driverSim = new Ps5ControllerSim(Controllers.kDriverPort, "Sim/Driver");
    private final Ps5ControllerSim operatorSim = new Ps5ControllerSim(Controllers.kOperatorPort, "Sim/Operator");

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
        configureAutos();
    }

    private void configureBindings() {
        drivetrain.register();
        drivetrain.registerTelemetry(logger::telemeterize);

        var driver = Controllers.getSwerveController();
        var operator = Controllers.getOPController();

        // Swerve: sticks, heading reset, place speed, pass aiming
        DriverButtons.bind(driver, drivetrain, swerveCommands, scoringCommands, drive, MaxSpeed, MaxAngularRate);

        // Shooter + intake on the operator controller. While aiming (R2) the driver stick still translates.
        OperatorButtons.bind(operator, scoringCommands, intakeCommands,
            () -> -driver.getLeftY() * MaxSpeed,
            () -> -driver.getLeftX() * MaxSpeed);

        // Rollers start when the arm reaches open and stop when it reaches closed (as in Catalyst).
        intakeCommands.bindRollersToArm();

        // The intake commands finish at their target, so the arm must hold where they leave it.
        linear_arm.setDefaultCommand(linear_arm.holdPosition());
    }

    private void configureAutos() {
        autoChooser.setDefaultOption("Drive to points", simpleAuto());
        autoChooser.addOption("Auto 1 (Catalyst)", auto1Starting.auto1());
        autoChooser.addOption("Nothing", Commands.none());
        SmartDashboard.putData("Auto chooser", autoChooser);
    }

    public void Periodic(){
        vision.periodic();
    }

    /** Call from Robot.simulationPeriodic(). */
    public void simulationPeriodic(){
        driverSim.periodic();
        operatorSim.periodic();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command simpleAuto() {
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

    public void resetRobot(){
        System.out.println(DriverStation.isDisabled());
        CommandScheduler.getInstance().schedule(
            Commands.parallel(            
            linear_arm.calibrateZero(),
            shooter.coast(),
            intakeRoller.coast(),
            drivetrain.applyRequest(()->new SwerveRequest.Idle()).withTimeout(1))
            .withTimeout(4)
            );
    }
}
