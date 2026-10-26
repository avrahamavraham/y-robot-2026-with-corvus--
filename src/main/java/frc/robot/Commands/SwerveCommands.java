// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.lang.annotation.Retention;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class SwerveCommands {
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentricFacingAngle followAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withHeadingPID(2.3, 0, 0)
    .withSteerRequestType(SteerRequestType.Position)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public SwerveCommands(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }
    public Command FollowHub(){
        return drivetrain.applyRequest(()->{
            return followAngle.withTargetDirection(
                drivetrain.getState().Pose.getTranslation().minus(Utils.hubPoseSupllier.get())
                .getAngle().plus(Rotation2d.k180deg));
        }).until(()->{
            return drivetrain.getState().Pose.getTranslation().minus(Utils.hubPoseSupllier.get()).getAngle().minus(drivetrain.getState().Pose.getRotation()).plus(Rotation2d.k180deg).getDegrees() < 4;
        }).andThen(drivetrain.applyRequest(SwerveRequest.SwerveDriveBrake::new));
    }
    public Command FollowPass(DoubleSupplier xAxis,DoubleSupplier yAxis){
        return drivetrain.applyRequest(()->{
            return followAngle.withVelocityX(xAxis.getAsDouble()).withVelocityY(yAxis.getAsDouble())
            .withTargetDirection(drivetrain.getState().Pose.getTranslation().minus(Utils.passPose.apply(drivetrain.getState().Pose)).getAngle().plus(Rotation2d.k180deg));
        });
        }
}
