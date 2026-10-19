// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class AutoFactory {
    // pid controllers
    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;
    // drive trian
    private final CommandSwerveDrivetrain drivetrain;
    
    private SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position)
    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    public AutoFactory(PIDController xController,PIDController yController,
    PIDController rotController,CommandSwerveDrivetrain drivetrain){
        this.xController = xController;
        this.yController = yController;
        this.rotController = rotController;
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        this.drivetrain = drivetrain;
    }

    private Command driveToPoint(Pose2d pose){
        return 
            drivetrain.applyRequest(()->{
                fieldCentric = fieldCentric.
                withVelocityX(-xController.calculate(drivetrain.getStateCopy().Pose.getX(), pose.getX())).
                withVelocityY(-yController.calculate(drivetrain.getStateCopy().Pose.getY(), pose.getY()))
                .withRotationalRate(rotController.calculate(drivetrain.getStateCopy().Pose.getRotation().getRadians(), 
                pose.getRotation().getRadians()));
                return fieldCentric;
            }
            ).until(()-> { 
                //change the numbers to your liking the smaller it is the better tune pid you need
                return Math.abs(xController.getError()) < 0.04
                && Math.abs(yController.getError()) < 0.04
                && Math.abs(rotController.getError()) < Math.toRadians(7);
            });
    }
    public Command driveToPoint(Pose2d... pose){
        var idle = new SwerveRequest.Idle();
        Command com = Commands.none();
        for (Pose2d pose2d : pose) {
            com = com.andThen(driveToPoint(pose2d));
        }
        return com.andThen(drivetrain.applyRequest(()->idle));
    }
}
