// Ported from yteam3211/robot2026Catalyst (Buttons/SwerveOp) onto the y-robot drivetrain.

package frc.robot.Buttons;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Commands.ScoringCommands;
import frc.robot.Commands.SwerveCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Driver controller (port 0). */
public final class DriverButtons {
  private DriverButtons() {}

  /**
   * <ul>
   *   <li>Left stick: translate, right stick X: rotate (field centric, default command)
   *   <li>Touchpad: reset field-centric heading (Catalyst: resetHeading)
   *   <li>D-pad up / down: place shot speed +5 / -5 rps (Catalyst bound both to up, a bug)
   *   <li>Cross (held): face the nearest pass target while driving (kept from the old Xbox A)
   * </ul>
   */
  public static void bind(
      CommandPS5Controller controller,
      CommandSwerveDrivetrain drivetrain,
      SwerveCommands swerveCommands,
      ScoringCommands scoring,
      SwerveRequest.FieldCentric drive,
      double maxSpeed,
      double maxAngularRate) {
    // Note: X is forward and Y is left in WPILib, so the stick axes are negated.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-controller.getLeftY() * maxSpeed)
                    .withVelocityY(-controller.getLeftX() * maxSpeed)
                    .withRotationalRate(-controller.getRightX() * maxAngularRate)));

    // Idle while disabled so the configured neutral mode is applied to the drive motors.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    controller.touchpad().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    controller.povUp().onTrue(scoring.adjustPlaceSpeed(ScoringCommands.kPlaceStepRps));
    controller.povDown().onTrue(scoring.adjustPlaceSpeed(-ScoringCommands.kPlaceStepRps));

    controller
        .cross()
        .whileTrue(
            swerveCommands.FollowPass(
                () -> -controller.getLeftY() * maxSpeed, () -> -controller.getLeftX() * maxSpeed));
  }
}
