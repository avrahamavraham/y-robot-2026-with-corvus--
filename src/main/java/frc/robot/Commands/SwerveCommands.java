// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    /** Heading controller gains ported from Catalyst's driveWithHeding (kP, kI, kD). */
    private static final double kHeadingP = 11.583;
    private static final double kHeadingI = 8;
    private static final double kHeadingD = 0;
    /** Catalyst slowed translation to 30% while it was aiming. */
    private static final double kAimTranslationScale = 0.3;

    private final SwerveRequest.FieldCentricFacingAngle driveWithHeadingRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withHeadingPID(kHeadingP, kHeadingI, kHeadingD)
    .withSteerRequestType(SteerRequestType.Position)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public SwerveCommands(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }
    public Command FollowHub(DoubleSupplier xAxis,DoubleSupplier yAxis){
        return drivetrain.applyRequest(()->{
            return followAngle.withVelocityX(xAxis.getAsDouble()).withVelocityY(yAxis.getAsDouble())
            .withTargetDirection(
                drivetrain.getState().Pose.getTranslation().minus(Utils.hubPoseSupllier.get())
                .getAngle().plus(Rotation2d.k180deg));
        });
    }
    public Command FollowPass(DoubleSupplier xAxis,DoubleSupplier yAxis){
        return drivetrain.applyRequest(()->{
            return followAngle.withVelocityX(xAxis.getAsDouble()).withVelocityY(yAxis.getAsDouble())
            .withTargetDirection(drivetrain.getState().Pose.getTranslation().minus(Utils.passPose.apply(drivetrain.getState().Pose)).getAngle());
        });
    }

    /**
     * Ported from Catalyst's {@code driveWithHeding.driveWithHeading}: drive field-centric while
     * the heading is held on {@code targetHeading}.
     *
     * @param xAxis field X velocity in m/s (already scaled by your max speed, same as FollowHub)
     * @param yAxis field Y velocity in m/s
     * @param targetHeading field-relative heading to hold, re-read every loop
     */
    public Command driveWithHeading(DoubleSupplier xAxis, DoubleSupplier yAxis, Supplier<Rotation2d> targetHeading){
        return drivetrain.applyRequest(()->
            driveWithHeadingRequest
                .withVelocityX(xAxis.getAsDouble() * kAimTranslationScale)
                .withVelocityY(yAxis.getAsDouble() * kAimTranslationScale)
                .withTargetDirection(targetHeading.get())
        ).withName("Swerve.DriveWithHeading");
    }
}
