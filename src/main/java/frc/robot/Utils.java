// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.lib.ShootOnTheMoveSolver;

/** Add your docs here. */
public class Utils {
    //shot on the move
    private static ShootOnTheMoveSolver shootOnTheMoveSolver;
    private static final InterpolatingDoubleTreeMap RPMmap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HoodDegreeMap = new InterpolatingDoubleTreeMap();
    public static Supplier<Translation2d> hubPoseSupllier = ()-> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    public static Supplier<Translation2d> passPose = 
        ()-> AllianceFlipUtil.apply(new Pose2d(2.5,5.5,Rotation2d.kZero)).getTranslation();
    private static Supplier<ShootOnTheMoveSolver.Solution> solutionSupplier= ()-> new ShootOnTheMoveSolver.
    Solution(0, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, false);
    public static DoubleSupplier rPMSupplier = ()->solutionSupplier.get().rpm();
    public static Supplier<Rotation2d> YawSupllier = ()-> solutionSupplier.get().chassisYaw();
    private static boolean isUpdated = false;
    private static ShootOnTheMoveSolver.Solution sol;
    public static void init(Subsystems sub){
        passPose = ()-> sub.drivetrain.getState().Pose.nearest(List.of(
        AllianceFlipUtil.apply(new Pose2d(2.5,5.5,Rotation2d.kZero)),
        AllianceFlipUtil.apply(new Pose2d(2.5,FieldConstants.fieldWidth - 5.5,Rotation2d.kZero)))).getTranslation();
            
        RPMmap.put(1.50, 2218.0);
        HoodDegreeMap.put(1.50, 62.0);
        RPMmap.put(1.75, 2272.0);
        HoodDegreeMap.put(1.75, 62.0);
        RPMmap.put(2.00, 2345.0);
        HoodDegreeMap.put(2.00, 62.0);
        RPMmap.put(2.25, 2427.0);
        HoodDegreeMap.put(2.25, 62.0);
        RPMmap.put(2.50, 2512.0);
        HoodDegreeMap.put(2.50, 62.0);
        RPMmap.put(2.75, 2599.0);
        HoodDegreeMap.put(2.75, 62.0);
        RPMmap.put(3.00, 2687.0);
        HoodDegreeMap.put(3.00, 62.0);
        RPMmap.put(3.25, 2775.0);
        HoodDegreeMap.put(3.25, 62.0);
        RPMmap.put(3.50, 2862.0);
        HoodDegreeMap.put(3.50, 62.0);
        RPMmap.put(3.75, 2949.0);
        HoodDegreeMap.put(3.75, 62.0);
        RPMmap.put(4.00, 3035.0);
        HoodDegreeMap.put(4.00, 62.0);
        RPMmap.put(4.25, 3120.0);
        HoodDegreeMap.put(4.25, 62.0);
        RPMmap.put(4.50, 3204.0);
        HoodDegreeMap.put(4.50, 62.0);
        RPMmap.put(4.75, 3288.0);
        HoodDegreeMap.put(4.75, 62.0);
        RPMmap.put(5.00, 3371.0);
        HoodDegreeMap.put(5.00, 62.0);
        shootOnTheMoveSolver = 
        new ShootOnTheMoveSolver(()-> sub.drivetrain.getState().Pose, 
        ()-> ChassisSpeeds.fromRobotRelativeSpeeds(sub.drivetrain.getState().Speeds, sub.drivetrain.getState().Pose.getRotation()), 
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
        sol = shootOnTheMoveSolver.calculate();
        solutionSupplier = ()->{
            if (!isUpdated) {
                sol = shootOnTheMoveSolver.calculate();
                isUpdated = true;
            }
            return sol;
        };
    }
    public static void UpdateLoop(){
        isUpdated = false;
    }
}

