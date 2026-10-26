// Ported from yteam3211/robot2026Catalyst (Buttons/devButton). Test bindings, NOT bound by
// default. Call DevButtons.bind(...) from RobotContainer in place of OperatorButtons when
// tuning, since it reuses the same operator buttons.

package frc.robot.Buttons;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.IntakeCommands;
import frc.robot.Commands.ScoringCommands;
import java.util.function.DoubleSupplier;

public final class DevButtons {
  private DevButtons() {}

  /**
   * <ul>
   *   <li>Triangle (held): spin the shooter to the SOTM rpm only (no aiming, no feeding)
   *   <li>Cross (held): aim at the hub while driving, no shooting
   *   <li>Square: stop rollers
   *   <li>Circle: toggle the intake arm
   *   <li>R2 (held): full shot, L2: toggle the intake (same as the operator layout)
   * </ul>
   */
  public static void bind(
      CommandPS5Controller controller,
      ScoringCommands scoring,
      IntakeCommands intake,
      DoubleSupplier driverX,
      DoubleSupplier driverY) {
    controller.triangle().whileTrue(scoring.shotHub());
    controller.cross().whileTrue(scoring.followHub(driverX, driverY));
    controller.square().onTrue(intake.stopRollers());
    controller.circle().onTrue(intake.toggle());
    controller.R2().whileTrue(scoring.shot(driverX, driverY));
    controller.L2().onTrue(intake.toggle());
  }
}
