// Ported from yteam3211/robot2026Catalyst (frc.robot.AutoCommands.Auto1Starting).
//
// Catalyst built smooth PathPlanner paths with DynamicAutoBuilder (path constraints, end
// rotation, end velocity). Neither that library nor PathPlanner is in this repo, so this version
// drives to the same poses with the existing AutoFactory (PID drive-to-point, stops at every
// pose). Same route, but not the same speed profile - the constraints and the pass-through end
// velocities are dropped. If you add the optional PathPlanner integration later, swap the bodies
// back to path following.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.function.Supplier;

public class Auto1Starting {
  /** Blue-alliance poses copied from Catalyst. */
  private static final Pose2d[] kAutoPoses =
      new Pose2d[] {
        new Pose2d(3.571, 5.292, Rotation2d.fromDegrees(125)),
        new Pose2d(5.931, 5.958, Rotation2d.fromDegrees(60)),
        new Pose2d(7.92, 7.423, Rotation2d.fromDegrees(-90)),
        new Pose2d(7.800, 4.581, Rotation2d.fromDegrees(-90)),
        new Pose2d(5.713, 5.073, Rotation2d.fromDegrees(-45)),
        new Pose2d(3.047, 5.838, Rotation2d.fromDegrees(-48.5)),
      };

  private final AutoFactory autoFactory;
  private final CommandSwerveDrivetrain drivetrain;

  public Auto1Starting(AutoFactory autoFactory, CommandSwerveDrivetrain drivetrain) {
    this.autoFactory = autoFactory;
    this.drivetrain = drivetrain;
  }

  /** Catalyst auto1(): poses 0 -> 1 -> 2, then 3, then 4 -> 5. Flipped for red at run time. */
  public Command auto1() {
    // Alliance is only known once the auto starts, so build the poses then, not at construction.
    return Commands.defer(
            () ->
                autoFactory
                    .driveToPoint(flip(kAutoPoses[0]), flip(kAutoPoses[1]), flip(kAutoPoses[2]))
                    // Catalyst's second segment started at pose 2 again; we are already there.
                    .andThen(autoFactory.driveToPoint(flip(kAutoPoses[3])))
                    .andThen(autoFactory.driveToPoint(flip(kAutoPoses[4]), flip(kAutoPoses[5]))),
            Set.of(drivetrain))
        .withName("Auto1Starting.auto1");
  }

  /** Catalyst driveToPose(): drive to a pose that is read when the command starts (no flipping). */
  public Command driveToPose(Supplier<Pose2d> pose) {
    return Commands.defer(() -> autoFactory.driveToPoint(pose.get()), Set.of(drivetrain))
        .withName("Auto1Starting.driveToPose");
  }

  private static Pose2d flip(Pose2d bluePose) {
    return AllianceFlipUtil.apply(bluePose);
  }
}
