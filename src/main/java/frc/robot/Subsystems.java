// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.ShootOnTheMoveSolver;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LinearArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/** Add your docs here. */
public class Subsystems {
    //shot on the move
    public final ShootOnTheMoveSolver shootOnTheMoveSolver;
    private final InterpolatingDoubleTreeMap RPMmap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap HoodDegreeMap = new InterpolatingDoubleTreeMap();
    //subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LinearArm linear_arm = new LinearArm();
    public final Shooter shooter = new Shooter();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public final Indexer indexer = new Indexer();
    public final Kicker kicker = new Kicker();
    public final Vision vision;

    public Subsystems(){
        RPMmap.put(1.00, 1565.0);
        HoodDegreeMap.put(1.00, 62.0);
        RPMmap.put(1.25, 1565.0);
        HoodDegreeMap.put(1.25, 62.0);
        RPMmap.put(1.50, 1573.0);
        HoodDegreeMap.put(1.50, 62.0);
        RPMmap.put(1.75, 1611.0);
        HoodDegreeMap.put(1.75, 62.0);
        RPMmap.put(2.00, 1663.0);
        HoodDegreeMap.put(2.00, 62.0);
        RPMmap.put(2.25, 1720.0);
        HoodDegreeMap.put(2.25, 62.0);
        RPMmap.put(2.50, 1781.0);
        HoodDegreeMap.put(2.50, 62.0);
        RPMmap.put(2.75, 1842.0);
        HoodDegreeMap.put(2.75, 62.0);
        RPMmap.put(3.00, 1904.0);
        HoodDegreeMap.put(3.00, 62.0);
        RPMmap.put(3.25, 1966.0);
        HoodDegreeMap.put(3.25, 62.0);
        RPMmap.put(3.50, 2028.0);
        HoodDegreeMap.put(3.50, 62.0);
        RPMmap.put(3.75, 2090.0);
        HoodDegreeMap.put(3.75, 62.0);
        RPMmap.put(4.00, 2150.0);
        HoodDegreeMap.put(4.00, 62.0);
        RPMmap.put(4.25, 2211.0);
        HoodDegreeMap.put(4.25, 62.0);
        RPMmap.put(4.50, 2270.0);
        HoodDegreeMap.put(4.50, 62.0);
        RPMmap.put(4.75, 2330.0);
        HoodDegreeMap.put(4.75, 62.0);
        RPMmap.put(5.00, 2388.0);
        HoodDegreeMap.put(5.00, 62.0);

        shootOnTheMoveSolver = 
        new ShootOnTheMoveSolver(()-> drivetrain.getState().Pose, 
        ()-> ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation()), 
        Utils.hubPoseSupllier, 
        new ShootOnTheMoveSolver.Config()
        .clampStrategy(ShootOnTheMoveSolver.ClampStrategy.LEAST_SQUARES)
        .efficiency(0.9)
        .hoodAngleDegMap(HoodDegreeMap)
        .rpmMap(RPMmap)
        .hoodLimitsDeg(62, 62)
        .rpmLimits(1000, 5500)
        .shooterOffset(0.16222, 0.00944)
        .wheelDiameterMeters(Inch.of(3).in(Meter))
        .shooterFacingDeg(180));
        if (Robot.isReal()) {
            vision = new Vision(drivetrain, 
            new VisionIOLimelight("limelight-gg", ()->drivetrain.getState().Pose.getRotation(), 0.2));
        } else {
            vision = new Vision(drivetrain,
            new VisionIOPhotonVisionSim("limelight-gg", 
                new Transform3d(-0.1808, -0.023073, 0.44788, new Rotation3d(0,0,Math.PI)), 
                ()->drivetrain.getState().Pose, 0.2)            
            );
        }
    }
}
