// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;

/** Add your docs here. */
public class Utils {
    public static Supplier<Translation2d> hubPoseSupllier = ()-> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    public static Function<Pose2d,Translation2d> passPose = 
        (pose)-> pose.nearest(List.of(
            AllianceFlipUtil.apply(new Pose2d(2.5,5.5,Rotation2d.kZero)),
            AllianceFlipUtil.apply(new Pose2d(2.5,FieldConstants.fieldWidth - 5.5,Rotation2d.kZero))))
            .getTranslation();
}
