// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class ShooterCommnads {
    private final Shooter shooter;
    private final CommandSwerveDrivetrain swerveDrivetrain;
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final Translation2d hubPose = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    public ShooterCommnads(Shooter shooter,CommandSwerveDrivetrain drivetrain){
        this.shooter = shooter;
        this.swerveDrivetrain = drivetrain;
        //add a full shooter map
    }
    public Command ShotAtHub(){
        return shooter.drive(()->rpmMap.get(swerveDrivetrain.getState().Pose.getTranslation().getDistance(hubPose)));
    }
}
