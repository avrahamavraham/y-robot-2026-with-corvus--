// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.simulation.PS5ControllerSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class Controllers {
    public static CommandPS5Controller driveController = new CommandPS5Controller(0);
    public static CommandPS5Controller opertorController = new CommandPS5Controller(1);
    private static Map<CommandXboxController, PS5ControllerSim> SimControllerMap = new HashMap<>();
    public static void SimInit(int port){
        SimControllerMap.put(new CommandXboxController(port), new PS5ControllerSim(port));
    }
    public static void bindXboxToPS5Sim() {
        SimControllerMap.forEach((xbox,sim)->{
                
    // A -> Cross
    xbox.a().onTrue(Commands.runOnce(() -> sim.setCrossButton(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setCrossButton(false)).ignoringDisable(true));

    // B -> Circle
    xbox.b().onTrue(Commands.runOnce(() -> sim.setCircleButton(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setCircleButton(false)).ignoringDisable(true));

    // X -> Square
    xbox.x().onTrue(Commands.runOnce(() -> sim.setSquareButton(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setSquareButton(false)).ignoringDisable(true));

    // Y -> Triangle
    xbox.y().onTrue(Commands.runOnce(() -> sim.setTriangleButton(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setTriangleButton(false)).ignoringDisable(true));

    // Left bumper -> L1
    xbox.leftBumper().onTrue(Commands.runOnce(() -> sim.setL1Button(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setL1Button(false)).ignoringDisable(true));

    // Right bumper -> R1
    xbox.rightBumper().onTrue(Commands.runOnce(() -> sim.setR1Button(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setR1Button(false)).ignoringDisable(true));

    // Back -> Create
    xbox.back().onTrue(Commands.runOnce(() -> sim.setCreateButton(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setCreateButton(false)).ignoringDisable(true));

    // Start -> Options
    xbox.start().onTrue(Commands.runOnce(() -> sim.setOptionsButton(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setOptionsButton(false)).ignoringDisable(true));

    // Left stick press -> L3
    xbox.leftStick().onTrue(Commands.runOnce(() -> sim.setL3Button(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setL3Button(false)).ignoringDisable(true));

    // Right stick press -> R3
    xbox.rightStick().onTrue(Commands.runOnce(() -> sim.setR3Button(true)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sim.setR3Button(false)).ignoringDisable(true));

    // Triggers -> L2 / R2 (button + axis)
    xbox.leftTrigger().onTrue(Commands.runOnce(() -> {
                sim.setL2Button(true);
                sim.setL2Axis(1.0);
            }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> {
                sim.setL2Button(false);
                sim.setL2Axis(-1.0);
            }).ignoringDisable(true));

    xbox.rightTrigger().onTrue(Commands.runOnce(() -> {
                sim.setR2Button(true);
                sim.setR2Axis(1.0);
            }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> {
                sim.setR2Button(false);
                sim.setR2Axis(-1.0);
            }).ignoringDisable(true));
        });
    }
    public static void simulationPeriodic(){ 
        SimControllerMap.forEach((xbox,sim)->{        
                sim.setLeftX(xbox.getLeftX()); 
                sim.setLeftY(xbox.getLeftY()); 
                sim.setRightX(xbox.getRightX()); 
                sim.setRightY(xbox.getRightY());
        });
    }
}
