// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controllers;
import frc.robot.RobotContainer;
import frc.robot.Subsystems;
import frc.robot.Utils;
import frc.robot.subsystems.LinearArm;

/** Add your docs here. */
public class CommandCollection {
    Subsystems sub;
    public CommandCollection(Subsystems sub) {
        this.sub = sub;
    }
    public Command runShooterToRpmHub() {
        return sub.shooter.drive(Utils.rPMSupplier);
    }
    public Command runAlignHub() {
        SwerveRequest.FieldCentricFacingAngle facingAngle = new FieldCentricFacingAngle()
            .withHeadingPID(2.3, 0, 0);
        return sub.drivetrain.applyRequest(() -> {
            return facingAngle
                .withTargetDirection(Utils.YawSupllier.get().plus(Rotation2d.k180deg))
                .withVelocityX(0)
                .withVelocityY(0);
        });
    }
    public Command runFullShot() {return runAlignHub().alongWith(runShooterToRpmHub())
        .finallyDo(sub.shooter.drive(()-> 0)::execute);}
    public Command runFullShotAuto() {return runAlignHub().alongWith(runShooterToRpmHub())
        .withTimeout(5).finallyDo(sub.shooter.drive(()-> 0)::execute);} 
    public Command ChangeIntake(){
       return Commands.runOnce(()->{
            if (RobotContainer.LinearArmsetpoint == LinearArm.Setpoint.in) {
                RobotContainer.LinearArmsetpoint = LinearArm.Setpoint.out;
            }
            else{
                RobotContainer.LinearArmsetpoint = LinearArm.Setpoint.in;
            }
        });
    }
    public Command runShooterToRpmPass(){
        return sub.shooter.drive(Utils.rPMSupplierPass);
    }
    public Command runAlignPass(){
        SwerveRequest.FieldCentricFacingAngle facingAngle = new FieldCentricFacingAngle()
            .withHeadingPID(2.3, 0, 0);
        return sub.drivetrain.applyRequest(() -> {
            return facingAngle
                .withTargetDirection(Utils.YawSupllierPass.get().plus(Rotation2d.k180deg))
                .withVelocityX(-Controllers.driveController.getLeftY())
                .withVelocityY(-Controllers.driveController.getLeftX());    
        });
    }
    public Command runFullShotPass() {return runAlignPass().alongWith(runShooterToRpmPass());}

}
