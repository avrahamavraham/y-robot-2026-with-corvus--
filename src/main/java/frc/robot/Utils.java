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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.lib.ShootOnTheMoveSolver;

/** Add your docs here. */
public class Utils {
    //shot on the move
    private static Subsystems subs;
    private static ShootOnTheMoveSolver shootOnTheMoveSolver;
    public static Supplier<Translation2d> hubPoseSupllier = ()-> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    public static Supplier<Translation2d> passPose =
        ()-> AllianceFlipUtil.apply(new Pose2d(2.5,5.5,Rotation2d.kZero)).getTranslation();

    // The solver reuses one Solution object, so this is the same object every loop.
    private static ShootOnTheMoveSolver.Solution sol = new ShootOnTheMoveSolver.Solution();
    private static Rotation2d yaw = Rotation2d.kZero;
    private static boolean isUpdated = false;

    public static DoubleSupplier rPMSupplier = ()-> { refresh(); return sol.rpm(); };
    public static Supplier<Rotation2d> YawSupllier = ()-> { refresh(); return yaw; };

    /** Solves at most once per loop, and only when something reads rPMSupplier / YawSupllier. */
    private static void refresh() {
        if (isUpdated || subs == null) {
            return;
        }
        var st = subs.drivetrain.getState(); // single call
        Rotation2d r = st.Pose.getRotation();
        double c = r.getCos();
        double s = r.getSin();
        ChassisSpeeds v = st.Speeds; // robot-relative -> field-relative by hand
        Translation2d hub = hubPoseSupllier.get();
        sol = shootOnTheMoveSolver.calculate(
            st.Pose.getX(), st.Pose.getY(), r.getRadians(),
            v.vxMetersPerSecond * c - v.vyMetersPerSecond * s,
            v.vxMetersPerSecond * s + v.vyMetersPerSecond * c,
            v.omegaRadiansPerSecond,
            hub.getX(), hub.getY());
        yaw = sol.chassisYaw(); // one small allocation per loop, only while shooting
        isUpdated = true;
    }

    public static void init(Subsystems sub){
        subs = sub;
        passPose = ()-> sub.drivetrain.getState().Pose.nearest(List.of(
        AllianceFlipUtil.apply(new Pose2d(2.5,5.5,Rotation2d.kZero)),
        AllianceFlipUtil.apply(new Pose2d(2.5,FieldConstants.fieldWidth - 5.5,Rotation2d.kZero)))).getTranslation();

        shootOnTheMoveSolver = new ShootOnTheMoveSolver(
        new ShootOnTheMoveSolver.Config()
        .clampStrategy(ShootOnTheMoveSolver.ClampStrategy.LEAST_SQUARES)
        .fixedHoodDeg(62)
        .shot(1.40, 2250)
        .shot(1.50, 2256)
        .shot(1.60, 2270)
        .shot(1.70, 2290)
        .shot(1.80, 2314)
        .shot(1.90, 2341)
        .shot(2.00, 2370)
        .shot(2.10, 2400)
        .shot(2.20, 2432)
        .shot(2.30, 2464)
        .shot(2.40, 2497)
        .shot(2.50, 2531)
        .shot(2.60, 2565)
        .shot(2.70, 2599)
        .shot(2.80, 2634)
        .shot(2.90, 2668)
        .shot(3.00, 2703)
        .shot(3.10, 2738)
        .shot(3.20, 2772)
        .shot(3.30, 2807)
        .shot(3.40, 2842)
        .shot(3.50, 2876)
        .shot(3.60, 2911)
        .shot(3.70, 2945)
        .shot(3.80, 2979)
        .shot(3.90, 3013)
        .shot(4.00, 3047)
        .shot(4.10, 3081)
        .shot(4.20, 3115)
        .shot(4.30, 3149)
        .shot(4.40, 3182)
        .shot(4.50, 3216)
        .shot(4.60, 3249)
        .shot(4.70, 3283)
        .shot(4.80, 3316)
        .shot(4.90, 3349)
        .shot(5.00, 3382)
        .rpmLimits(1000, 5500)
        .shooterOffset(0.16222, 0.00944)
        .wheelDiameterMeters(Inch.of(3).in(Meter))
        .shooterFacingDeg(180));
    }
    public static void UpdateLoop(){
        isUpdated = false;
    }
}