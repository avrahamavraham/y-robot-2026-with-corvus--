// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.file.LinkOption;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.LinearArm;

/** Add your docs here. */
public class IntakeCommands {
    private IntakeRoller intakeRoller;
    private LinearArm linearArm;

    public IntakeCommands(IntakeRoller intakeRoller,LinearArm linearArm){
        this.intakeRoller = intakeRoller;
        this.linearArm = linearArm;
    }
    public Command StartRoller(){
        return intakeRoller.drive(()->IntakeRoller.Setpoint.Run);
    }
    public Command StopRoller(){
        return intakeRoller.drive(()->IntakeRoller.Setpoint.Stop);
    }
    private LinearArm.Setpoint intakeSetpoint = LinearArm.Setpoint.out;
    // public Command OpenIntake(){
        
    // }

}
