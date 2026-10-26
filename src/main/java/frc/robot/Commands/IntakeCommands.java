// Ported from yteam3211/robot2026Catalyst (CommandCollections + IntakeLinear + IntakeRoller).

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.LinearArm;

/**
 * Intake arm + roller commands.
 *
 * <p>Naming: Catalyst calls the arm positions "open" (0.117561 m) and "close" (0 m). In this repo
 * the same two numbers are {@code LinearArm.Setpoint.in} (0.117561) and {@code
 * LinearArm.Setpoint.out} (0), so open == {@code in} and close == {@code out}. The constants are
 * identical, only the labels differ - double check on the robot that "open" really deploys the
 * intake.
 *
 * <p>The arm commands FINISH once the arm is at the target (like Catalyst's goTo). After they
 * finish, the arm's default command takes over, so set it to {@code linearArm.holdPosition()}
 * instead of a fixed setpoint, otherwise the arm drives back to the default setpoint.
 */
public class IntakeCommands {
  // ----- values copied from Catalyst -----
  /** RollerMechanism intakeSpeed(0.7), duty cycle. */
  public static final double kIntakeDutyCycle = 0.7;
  /** RollerMechanism ejectSpeed(-0.3), duty cycle. */
  public static final double kEjectDutyCycle = -0.3;
  /** Catalyst jogUp(5) / jogDown(-5): volts. Converted to duty cycle for LinearArm.manualDrive. */
  public static final double kJogVolts = 5.0;
  /** Catalyst IntakeLinear positionTolerance(0.01) meters. */
  private static final double kPositionToleranceMeters = 0.01;
  /** Same drum radius as LinearArm (it is private there). Used to turn meters into radians. */
  private static final double kDrumRadiusMeters = 0.0159;
  /** Safety so a stalled arm can never hang an auto sequence. Not in Catalyst. */
  private static final double kArmMoveTimeoutSeconds = 2.0;

  private final LinearArm arm;
  private final IntakeRoller roller;

  /** True when the arm is at the deployed ("open" in Catalyst) position. */
  public final Trigger armAtOpen = new Trigger(() -> armAt(LinearArm.Setpoint.in));
  /** True when the arm is at the stowed ("close" in Catalyst) position. */
  public final Trigger armAtClose = new Trigger(() -> armAt(LinearArm.Setpoint.out));

  public IntakeCommands(LinearArm arm, IntakeRoller roller) {
    this.arm = arm;
    this.roller = roller;
  }

  private boolean armAt(LinearArm.Setpoint setpoint) {
    return arm.getPosition()
        .isNear(setpoint.target, Radians.of(kPositionToleranceMeters / kDrumRadiusMeters));
  }

  private Command goTo(LinearArm.Setpoint setpoint) {
    return arm.goToSetpoint(() -> setpoint)
        .until(() -> armAt(setpoint))
        .withTimeout(kArmMoveTimeoutSeconds);
  }

  // ----- arm -----

  /** Catalyst IntakeLinear.open(). */
  public Command open() {
    return goTo(LinearArm.Setpoint.in).withName("Intake.Open");
  }

  /** Catalyst IntakeLinear.close(). */
  public Command close() {
    return goTo(LinearArm.Setpoint.out).withName("Intake.Close");
  }

  /** Catalyst CommandCollections.Intake(): if the arm is open close it, otherwise open it. */
  public Command toggle() {
    return Commands.either(close(), open(), armAtOpen).withName("Intake.Toggle");
  }

  /** Catalyst OpenIntakeDumb(): jog toward open while held (no position control). */
  public Command jogOpen() {
    return arm.manualDrive(() -> kJogVolts / 12.0).withName("Intake.JogOpen");
  }

  /** Catalyst CloseIntakeDump(): jog toward close while held (no position control). */
  public Command jogClose() {
    return arm.manualDrive(() -> -kJogVolts / 12.0).withName("Intake.JogClose");
  }

  // ----- rollers -----

  /** Catalyst startRollers(). Runs until another roller command replaces it. */
  public Command startRollers() {
    return roller.driveDutyCycle(() -> kIntakeDutyCycle).withName("Intake.StartRollers");
  }

  /** Catalyst ejectRoller(). Coasts when released. */
  public Command ejectRollers() {
    return roller.driveDutyCycle(() -> kEjectDutyCycle).withName("Intake.EjectRollers");
  }

  /** Catalyst stopRoller(). */
  public Command stopRollers() {
    return roller.coast().withName("Intake.StopRollers");
  }

  /**
   * Catalyst IntakeRoller.get() wired this inside the subsystem: rollers start as soon as the arm
   * reaches open and stop as soon as it reaches close. Call once from RobotContainer if you want
   * that behavior.
   */
  public void bindRollersToArm() {
    armAtOpen.onTrue(startRollers());
    armAtClose.onTrue(stopRollers());
  }
}
