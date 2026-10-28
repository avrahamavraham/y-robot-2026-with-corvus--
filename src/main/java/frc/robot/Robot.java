// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // java.io.File f = new java.io.File("logs/check/check.hoot");
        // System.out.println("cwd=" + new java.io.File(".").getAbsolutePath());
        // System.out.println("exists=" + f.exists() + " size=" + f.length());
        // try {
        //     System.out.println("status=" + HootReplay.loadFile(f.getPath()));
        // } catch (IllegalArgumentException e) {
        //     System.out.println("loadFile threw: " + e.getMessage());
        // }
        // System.out.println("loaded=" + HootReplay.isFileLoaded());
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        checkMemory();
        // System.out.println(HootReplay.isFileLoaded());
        m_timeAndJoystickReplay.update();
        m_robotContainer.Periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        m_robotContainer.resetRobot();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        Controllers.simulationPeriodic();
    }


    /* low heap warning: alert turns on below kLowMemFraction available, off again above kRecoverMemFraction */
    private static final double kLowMemFraction = 0.10;
    private static final double kRecoverMemFraction = 0.15;
    private static final double kMemWarnCooldownSec = 5.0;
    private final Alert m_lowMemoryAlert = new Alert("Low heap memory", AlertType.kWarning);
    private double m_lastMemWarnTime = 0.0;
 
    /** Warns (dashboard alert + throttled Driver Station warning) when the Java heap is nearly full. */
    private void checkMemory() {
        Runtime rt = Runtime.getRuntime();
        long max = rt.maxMemory();
        // System.out.println(rt.availableProcessors());
        // System.out.println(max/ 1_048_576);
        long available = max - (rt.totalMemory() - rt.freeMemory());
        // System.out.println(available / 1_048_576);
        double fraction = (double) available / max;
 
        if (fraction < kLowMemFraction) {
            double now = Timer.getFPGATimestamp();
            if (!m_lowMemoryAlert.get() || now - m_lastMemWarnTime > kMemWarnCooldownSec) {
                m_lastMemWarnTime = now;
                String msg = "Low heap memory: " + available / 1_048_576 + 
                " MB available of "
                    + max / 1_048_576 + " MB";
                m_lowMemoryAlert.setText(msg);
                DriverStation.reportWarning(msg, false);
            }
            m_lowMemoryAlert.set(true);
        } else if (fraction > kRecoverMemFraction) {
            m_lowMemoryAlert.set(false);
        }
    }

    @Override
    public void simulationInit() {
        Controllers.SimInit(0);
        Controllers.SimInit(1);
        Controllers.bindXboxToPS5Sim();
    }
}
