// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;

/** Add your docs here. */
public class Utils {
    public static Supplier<Translation2d> hubPoseSupllier = ()-> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
}
