// Ported from yteam3211/robot2026Catalyst (CommandCollections: ShotHub, FollowHub, Shot, ShotAuto,
// ShotFromPerPalce, ejectIndexerKicer, MoveBallsToShooter).

package frc.robot.Commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShootOnTheMove.SOTM;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Shooting commands: spin up from the shoot-on-the-move solution, aim at the hub, and feed balls
 * (kicker then indexer) only when the shooter is at speed and the robot is aimed.
 *
 * <p>Catalyst did the feeding with global triggers registered once at boot
 * (MoveBallsToShooter). Here the feeding is part of the shoot commands, so nothing runs unless a
 * shoot command is scheduled. All speeds are rotations per second.
 */
public class ScoringCommands {
  // ----- values copied from Catalyst (rotations per second) -----
  public static final double kKickerFeedRps = 66;
  public static final double kIndexerFeedRps = 17;
  // NOTE: these two look swapped in Catalyst (indexer -75, kicker -17) but are copied as-is.
  public static final double kIndexerEjectRps = -75;
  public static final double kKickerEjectRps = -17;
  /** Catalyst velocityTolerance for Shooter / Kicker. */
  private static final double kShooterToleranceRps = 5;
  private static final double kKickerToleranceRps = 2;
  /** Catalyst fired when |heading - target| < 3 degrees. */
  private static final double kAimToleranceDegrees = 3;
  /** Catalyst ShotAuto(): withTimeout(8). */
  public static final double kAutoShotSeconds = 8;
  /** Catalyst "Shot" named command: close the intake 3 s into the shot. */
  public static final double kAutoCloseIntakeDelaySeconds = 3;
  /** Catalyst PlaceRpm (the value is passed to spinUp so it is really rotations per second). */
  private static final double kInitialPlaceRps = 35;
  /** Catalyst changed the place speed in steps of 5. */
  public static final double kPlaceStepRps = 5;

  private final Shooter shooter;
  private final Kicker kicker;
  private final Indexer indexer;
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveCommands swerveCommands;
  private final SOTM sotm;

  private final Trigger shooterAtSpeed;
  private final Trigger kickerAtSpeed;

  private double placeRps = kInitialPlaceRps;

  public ScoringCommands(
      Shooter shooter,
      Kicker kicker,
      Indexer indexer,
      CommandSwerveDrivetrain drivetrain,
      SwerveCommands swerveCommands,
      SOTM sotm) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
    this.drivetrain = drivetrain;
    this.swerveCommands = swerveCommands;
    this.sotm = sotm;
    shooterAtSpeed = shooter.createIsNearTargetTrigger(RotationsPerSecond.of(kShooterToleranceRps));
    kickerAtSpeed = kicker.createIsNearTargetTrigger(RotationsPerSecond.of(kKickerToleranceRps));
  }

  // ----- building blocks -----

  /** True when the robot heading is within tolerance of the SOTM aim angle (wrap-safe). */
  public boolean isAimedAtHub() {
    double errorDegrees =
        drivetrain.getStateCopy().Pose.getRotation().minus(sotm.hubDriveAngle()).getDegrees();
    return Math.abs(errorDegrees) < kAimToleranceDegrees;
  }

  /**
   * Catalyst MoveBallsToShooter(), as a command: kicker runs when the shooter is at speed and
   * {@code canFire}; the indexer runs when the kicker is also at speed. Ends never; cancel it.
   */
  public Command feedWhenReady(BooleanSupplier canFire) {
    BooleanSupplier fireNow = () -> shooterAtSpeed.getAsBoolean() && canFire.getAsBoolean();
    DoubleSupplier kickerRps = () -> fireNow.getAsBoolean() ? kKickerFeedRps : 0.0;
    DoubleSupplier indexerRps =
        () ->
            fireNow.getAsBoolean() && kickerAtSpeed.getAsBoolean() ? kIndexerFeedRps : 0.0;
    return Commands.parallel(kicker.drive(kickerRps), indexer.drive(indexerRps))
        .withName("Scoring.FeedWhenReady");
  }

  // ----- hub shooting (shoot on the move) -----

  /** Catalyst ShotHub(): spin the shooter to the SOTM rpm. Spin only, does not feed. */
  public Command shotHub() {
    // Catalyst wrapped this in a DeferredCommand (rpm read once). Reading it every loop is what
    // makes shoot-on-the-move actually track the moving robot.
    return shooter.drive(sotm::hubRps).withName("Scoring.ShotHub");
  }

  /** Catalyst FollowHub(): face the SOTM aim angle while driving with the given field speeds. */
  public Command followHub(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    return swerveCommands.driveWithHeading(xAxis, yAxis, sotm::hubDriveAngle);
  }

  /** Catalyst FollowHub() with no driver input (used by autos). */
  public Command followHub() {
    return followHub(() -> 0, () -> 0);
  }

  /**
   * Catalyst Shot() plus its feeding: aim, spin up and feed. Feeds only when the SOTM solution is
   * valid (Catalyst fed even with an invalid, 0 rpm solution), the shooter is at speed and the
   * robot is aimed.
   */
  public Command shot(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    return Commands.parallel(
            shotHub(),
            followHub(xAxis, yAxis),
            feedWhenReady(() -> sotm.hubShotValid() && isAimedAtHub()))
        .withName("Scoring.Shot");
  }

  /** Catalyst Shot() with no driver input. */
  public Command shot() {
    return shot(() -> 0, () -> 0);
  }

  /** Catalyst ShotAuto(): the shot for 8 seconds. */
  public Command shotAuto() {
    return shot().withTimeout(kAutoShotSeconds).withName("Scoring.ShotAuto");
  }

  /**
   * Catalyst "Shot" PathPlanner named command: ShotAuto while closing the intake 3 s in.
   *
   * @param intake used for the close command
   */
  public Command shotAutoAndCloseIntake(IntakeCommands intake) {
    return shotAuto()
        .alongWith(Commands.waitSeconds(kAutoCloseIntakeDelaySeconds).andThen(intake.close()))
        .withName("Scoring.ShotAutoAndCloseIntake");
  }

  // ----- shooting from a fixed spot -----

  /**
   * Catalyst ShotFromPerPalce() with the feeding its trigger did: spin to the place speed and feed
   * as soon as the shooter is at speed. No aiming, no SOTM (Catalyst's R1 override).
   */
  public Command shotFromPlace() {
    return Commands.parallel(shooter.drive(() -> placeRps), feedWhenReady(() -> true))
        .withName("Scoring.ShotFromPlace");
  }

  /** Change the place speed (Catalyst used +/-5 on the driver d-pad). */
  public Command adjustPlaceSpeed(double deltaRps) {
    return Commands.runOnce(() -> placeRps += deltaRps).ignoringDisable(true);
  }

  public double getPlaceRps() {
    return placeRps;
  }

  // ----- eject -----

  /** Catalyst ejectIndexerKicer(). */
  public Command ejectIndexerKicker() {
    return Commands.parallel(indexer.drive(() -> kIndexerEjectRps), kicker.drive(() -> kKickerEjectRps))
        .withName("Scoring.EjectIndexerKicker");
  }
}
