// Ported from yteam3211/robot2026Catalyst (frc.robot.ShootOnTheMove.SOTM).
//
// Differences from the Catalyst version:
//  - Instance based (needs the CommandSwerveDrivetrain) instead of reaching for Swerve.get().
//  - The ballistic LUT is generated in the constructor, NOT in a static initializer. Build this
//    object while the robot is starting up (RobotContainer constructor) so the simulation never
//    runs in the middle of a match and causes a loop overrun.
//  - Hub position comes from Utils.hubPoseSupllier (alliance flipped) instead of a hard coded point.
//  - hubParameters() is cached per robot cycle, so calling it from several places in the same
//    loop only runs the solver (which has warm-start state) once.
//  - AdvantageKit Logger calls were replaced with SmartDashboard.

package frc.robot.ShootOnTheMove;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Millimeter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AllianceFlipUtil;
import frc.robot.ShootOnTheMove.ShotCalculator.LaunchParameters;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class SOTM {
  public static final Transform3d OFF_SET_SHOOTER =
      new Transform3d(
          Millimeter.of(-162.22),
          Millimeter.of(-9.44),
          Millimeter.of(546),
          new Rotation3d(Degree.of(180), Degree.of(0), Degree.of(0)));
  public static final Angle SHOOTER_ANGLE = Degree.of(62);

  /** Confidence fed to the solver. Catalyst used a constant 0.9; wire real vision data here. */
  private static final double VISION_CONFIDENCE = 0.9;

  private final CommandSwerveDrivetrain drivetrain;
  private final ShotCalculator hubCalculator;

  private LaunchParameters cached = LaunchParameters.INVALID;
  private double cachedAtSeconds = Double.NEGATIVE_INFINITY;

  public SOTM(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    ProjectileSimulator.SimParameters hubParams =
        new ProjectileSimulator.SimParameters(
            0.215, // ball mass kg
            0.1501, // ball diameter m
            0.47, // drag coeff (smooth sphere)
            0.2, // Magnus coeff
            1.225, // air density
            Millimeter.of(546).in(Meter), // exit height (m), floor to where the ball leaves
            Units.inchesToMeters(3), // flywheel diameter (m)
            1.83, // target height (m), from game manual
            0.83, // slip factor (0 = no grip, 1 = perfect). Higher = less power, lower = more
            62, // launch angle from horizontal, measure from CAD
            0.001, // sim timestep
            1500,
            6000,
            25,
            5.0 // RPM search range, iterations, max sim time
            );

    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = OFF_SET_SHOOTER.getX();
    config.launcherOffsetY = OFF_SET_SHOOTER.getY();
    config.phaseDelayMs = 20.0; // your vision pipeline latency
    config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
    config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar = 3.0; // heading tolerance tightens with robot speed
    config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub
    config.maxScoringDistance = 10.0;
    config.minScoringDistance = 0.1;
    config.maxSOTMSpeed = 3.0;
    config.headingMaxErrorRad = Units.degreesToRadians(2);
    config.shooterAngleOffsetRad = Math.PI;

    hubCalculator = new ShotCalculator(config);
    // This is the expensive part: it simulates the ball flight for a range of distances.
    var lut = new ProjectileSimulator(hubParams).generateLUT();
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        hubCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
      }
    }
  }

  /**
   * Hub shot solution for the current robot state. Safe to call many times per cycle; the solver
   * only runs once per FPGA timestamp.
   *
   * @return the solution, or a zero-rpm solution (that keeps the aim angle) when no valid shot
   *     exists. Check {@link LaunchParameters#isValid()} before feeding a ball.
   */
  public LaunchParameters hubParameters() {
    double now = Timer.getFPGATimestamp();
    if (now == cachedAtSeconds) {
      return cached;
    }
    cachedAtSeconds = now;

    var state = drivetrain.getStateCopy();
    // CTRE reports Speeds robot-relative; the solver wants field-relative too.
    ChassisSpeeds robotSpeeds = state.Speeds;
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, state.Pose.getRotation());

    Translation2d hubForward =
        AllianceFlipUtil.shouldFlip() ? new Translation2d(-1, 0) : new Translation2d(1, 0);

    var inputs =
        new ShotCalculator.ShotInputs(
            state.Pose,
            fieldSpeeds,
            robotSpeeds,
            Utils.hubPoseSupllier.get(),
            hubForward,
            VISION_CONFIDENCE,
            drivetrain.getPigeon2().getPitch().getValueAsDouble(),
            drivetrain.getPigeon2().getRoll().getValueAsDouble());

    LaunchParameters solution = hubCalculator.calculate(inputs);
    SmartDashboard.putBoolean("SOTM/Hub/Valid", solution.isValid());
    SmartDashboard.putNumber("SOTM/Hub/Rpm", solution.rpm());
    SmartDashboard.putNumber("SOTM/Hub/DriveAngleDeg", solution.driveAngle().getDegrees());
    SmartDashboard.putNumber("SOTM/Hub/Confidence", solution.confidence());

    if (solution.isValid()) {
      cached = solution;
    } else {
      // Keep aiming at the hub but ask for no shot.
      cached = new LaunchParameters(0, 0, solution.driveAngle(), 0, false, 0, 0, 0, false);
    }
    return cached;
  }

  /** Shooter target in rotations per second (what Shooter.drive(DoubleSupplier) expects). */
  public double hubRps() {
    return hubParameters().rpm() / 60.0;
  }

  /** Field-relative heading the robot should face to shoot at the hub. */
  public Rotation2d hubDriveAngle() {
    return hubParameters().driveAngle();
  }

  public boolean hubShotValid() {
    return hubParameters().isValid();
  }
}
