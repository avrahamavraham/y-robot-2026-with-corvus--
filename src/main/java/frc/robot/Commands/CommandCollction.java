// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.Utils;

/** Add your docs here. */
public class CommandCollction {
    Subsystems sub;
    public CommandCollction(Subsystems sub){
        this.sub = sub;
    }
    public Command ShooterToRpm(){
        return sub.shooter.drive(Utils.rPMSupplier);
    }
    public Command AutoAling(){
        SwerveRequest.FieldCentricFacingAngle facingAngle = new FieldCentricFacingAngle()
        .withHeadingPID(2.3, 0, 0);
        return sub.drivetrain.applyRequest(()->{
            return facingAngle.withTargetDirection(Utils.YawSupllier.get().plus(Rotation2d.k180deg))
            .withVelocityX(0)
            .withVelocityY(0);
        });
    }
    public Command FullShot(){
        return AutoAling().alongWith(ShooterToRpm());
    }
    public Command FullShotAuto(){
        return FullShot().withTimeout(5);
    }


}
