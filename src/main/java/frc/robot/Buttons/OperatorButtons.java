// Ported from yteam3211/robot2026Catalyst (Buttons/OPButtons).

package frc.robot.Buttons;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.IntakeCommands;
import frc.robot.Commands.ScoringCommands;
import java.util.function.DoubleSupplier;

/** Operator controller (port 1). */
public final class OperatorButtons {
  private OperatorButtons() {}

  /**
   * Shooter
   *
   * <ul>
   *   <li>R2 (held): aim at the hub with SOTM, spin up and feed. The driver sticks still translate
   *       the robot (slowed) while aiming.
   *   <li>R1 (held): shoot from the current spot at the place speed, no aiming
   *   <li>Triangle (held): eject through indexer and kicker
   * </ul>
   *
   * Intake
   *
   * <ul>
   *   <li>L2: toggle the intake arm open / closed
   *   <li>D-pad up / down (held): jog the arm open / closed
   *   <li>Circle: start rollers, Square: stop rollers, double-tap-and-hold Circle: eject rollers
   * </ul>
   *
   * @param driverX field X velocity in m/s from the driver stick, used while aiming
   * @param driverY field Y velocity in m/s from the driver stick, used while aiming
   */
  public static void bind(
      CommandPS5Controller controller,
      ScoringCommands scoring,
      IntakeCommands intake,
      DoubleSupplier driverX,
      DoubleSupplier driverY) {
    // shooter
    controller.R2().whileTrue(scoring.shot(driverX, driverY));
    controller.R1().whileTrue(scoring.shotFromPlace());
    controller.triangle().whileTrue(scoring.ejectIndexerKicker());

    // intake
    controller.L2().onTrue(intake.toggle());
    controller.povUp().whileTrue(intake.jogOpen());
    controller.povDown().whileTrue(intake.jogClose());
    controller.circle().onTrue(intake.startRollers());
    controller.square().onTrue(intake.stopRollers());
    controller.circle().multiPress(2, 0.5).whileTrue(intake.ejectRollers());
  }
}
