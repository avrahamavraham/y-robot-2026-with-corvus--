package frc.robot.sim;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Fake PS5 controller for simulation, driven from SmartDashboard / Glass / the sim GUI so you can
 * test every binding without a gamepad.
 *
 * <p>Each button shows up as a boolean at {@code <name>/<Button>} (e.g. {@code Sim/Operator/R2}),
 * the sticks as numbers ({@code LeftX, LeftY, RightX, RightY}, -1..1) and the d-pad as {@code
 * POV} (-1 = released, otherwise 0, 45, 90 ... 315 degrees). Nothing is written unless {@code
 * <name>/Enabled} is ticked, so a real or keyboard controller plugged into the same port is not
 * overridden by default. Call {@link #periodic()} from {@code simulationPeriodic}.
 *
 * <p>Uses {@link GenericHIDSim} with the {@link PS5Controller} button and axis numbers, so it
 * feeds the same inputs {@code CommandPS5Controller} reads.
 */
public class Ps5ControllerSim {
  private static final PS5Controller.Button[] kButtons = {
    PS5Controller.Button.kSquare,
    PS5Controller.Button.kCross,
    PS5Controller.Button.kCircle,
    PS5Controller.Button.kTriangle,
    PS5Controller.Button.kL1,
    PS5Controller.Button.kR1,
    PS5Controller.Button.kL2,
    PS5Controller.Button.kR2,
    PS5Controller.Button.kCreate,
    PS5Controller.Button.kOptions,
    PS5Controller.Button.kL3,
    PS5Controller.Button.kR3,
    PS5Controller.Button.kPS,
    PS5Controller.Button.kTouchpad,
  };

  private final GenericHIDSim sim;
  private final String key;

  /**
   * @param port the controller port to simulate (see {@link frc.robot.Controllers})
   * @param key SmartDashboard folder, e.g. "Sim/Operator"
   */
  public Ps5ControllerSim(int port, String key) {
    this.sim = new GenericHIDSim(port);
    this.key = key;
    SmartDashboard.putBoolean(key + "/Enabled", false);
    for (var button : kButtons) {
      SmartDashboard.putBoolean(key + "/" + name(button), false);
    }
    for (String axis : new String[] {"LeftX", "LeftY", "RightX", "RightY"}) {
      SmartDashboard.putNumber(key + "/" + axis, 0.0);
    }
    SmartDashboard.putNumber(key + "/POV", -1);
  }

  private static String name(PS5Controller.Button button) {
    // kCross -> Cross
    return button.name().substring(1);
  }

  /** Press or release a button from code (for scripted sim tests). */
  public void setButton(PS5Controller.Button button, boolean pressed) {
    sim.setRawButton(button.value, pressed);
    sim.notifyNewData();
  }

  /** Copy the dashboard values into the simulated controller. Call every simulation loop. */
  public void periodic() {
    if (!SmartDashboard.getBoolean(key + "/Enabled", false)) {
      return;
    }
    for (var button : kButtons) {
      sim.setRawButton(button.value, SmartDashboard.getBoolean(key + "/" + name(button), false));
    }
    sim.setRawAxis(PS5Controller.Axis.kLeftX.value, SmartDashboard.getNumber(key + "/LeftX", 0));
    sim.setRawAxis(PS5Controller.Axis.kLeftY.value, SmartDashboard.getNumber(key + "/LeftY", 0));
    sim.setRawAxis(PS5Controller.Axis.kRightX.value, SmartDashboard.getNumber(key + "/RightX", 0));
    sim.setRawAxis(PS5Controller.Axis.kRightY.value, SmartDashboard.getNumber(key + "/RightY", 0));
    sim.setPOV((int) SmartDashboard.getNumber(key + "/POV", -1));
    sim.notifyNewData();
  }
}
