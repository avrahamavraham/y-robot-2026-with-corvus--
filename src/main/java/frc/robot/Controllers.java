// Ported from yteam3211/robot2026Catalyst (frc.robot.Controllers).

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/** The two PS5 controllers: port 0 drives, port 1 operates. */
public final class Controllers {
  public static final int kDriverPort = 0;
  public static final int kOperatorPort = 1;

  private static final CommandPS5Controller driver = new CommandPS5Controller(kDriverPort);
  private static final CommandPS5Controller operator = new CommandPS5Controller(kOperatorPort);

  private Controllers() {}

  /** Catalyst getSwerveController(). */
  public static CommandPS5Controller getSwerveController() {
    return driver;
  }

  /** Catalyst getOPController(). */
  public static CommandPS5Controller getOPController() {
    return operator;
  }
}
