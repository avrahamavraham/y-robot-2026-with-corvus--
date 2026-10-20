// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootReplay;

import edu.wpi.first.wpilibj.TimedRobot;
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
        Runtime rt = Runtime.getRuntime();
        long used = rt.totalMemory() - rt.freeMemory();
        long max = rt.maxMemory();
        // if ( used / 1_048_576 > 512) {
            System.out.printf("Heap: %d MB used of %d MB max%n", used / 1_048_576, max / 1_048_576);        
        // }
        // System.out.println(HootReplay.isFileLoaded());
        m_timeAndJoystickReplay.update();
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
    public void simulationPeriodic() {}
}
