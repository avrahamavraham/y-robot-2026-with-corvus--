// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/** Add your docs here. */
public class FinalCommands {
    private ShooterCommnads shooterCommnads;
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;
    private CommandPS5Controller swerveController;
    public FinalCommands(ShooterCommnads shooterCommnads,IntakeCommands intakeCommands,SwerveCommands swerveCommands,CommandPS5Controller swerveController){
        this.shooterCommnads = shooterCommnads;
        this.intakeCommands = intakeCommands;
        this.swerveCommands = swerveCommands;
        this.swerveController = swerveController;
    }
    public Command Pass(){
        return swerveCommands.FollowPass(()-> -swerveController.getLeftY(), ()-> -swerveController.getLeftX())
        .alongWith(shooterCommnads.Pass());
    }
    public Command Shot(){
        return swerveCommands.FollowHub().andThen(shooterCommnads.ShotAtHub()).withTimeout(6);
    }
    public Command ShotFromPlace(){
        return shooterCommnads.ShotAtHubFromPlace();
    }
    // public Command 
}
