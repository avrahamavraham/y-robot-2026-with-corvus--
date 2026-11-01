package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

/**
 * Shoot-on-the-move solver using velocity subtraction (Huskie Physics, team 3061).
 *
 * <p>Idea: the static shot for the shooter's current position gives the ball velocity we want in the
 * FIELD frame. A moving robot adds its own velocity to the ball, so we subtract it and convert the
 * result back to flywheel RPM, hood pitch and yaw.
 *
 * <p>Conventions:
 *
 * <ul>
 *   <li>{@code rpmMap}: shooter-to-target distance (meters) -> flywheel RPM
 *   <li>{@code hoodAngleDegMap}: shooter-to-target distance (meters) -> launch elevation in degrees
 *       ABOVE HORIZONTAL (0 = flat, 90 = straight up)
 *   <li>Pose and speeds are FIELD-relative (CTRE swerve:
 *       {@code ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation())})
 *   <li>{@code shooterOffset}: shooter position in the robot frame (x forward, y left), meters
 * </ul>
 */
public class ShootOnTheMoveSolver {

  /** What to do when the exact solution needs a hood angle outside the hood limits. */
  public enum ClampStrategy {
    /**
     * Pin the hood, pick the speed that matches the required HORIZONTAL velocity. Vertical velocity
     * ends up off (time of flight changes).
     */
    MATCH_HORIZONTAL,
    /**
     * Pin the hood, pick the speed that minimizes the 3D velocity error (projection of the required
     * vector on the pinned launch direction). Splits the error between horizontal and vertical.
     */
    LEAST_SQUARES
  }

  /**
   * @param rpm flywheel speed in RPM
   * @param hoodPitch launch elevation above horizontal
   * @param fieldYaw direction the SHOOTER must point, FIELD frame
   * @param robotYaw direction the shooter must point relative to the robot's forward axis (turret
   *     target, before any turret mounting offset)
   * @param chassisYaw FIELD-frame heading the CHASSIS must have so a fixed shooter (facing
   *     {@code shooterFacing} relative to robot forward) points along {@code fieldYaw}; use this as
   *     the target for fixed-shooter heading control
   * @param achievable false if the hood or RPM had to be clamped, so this shot will not follow the
   *     intended trajectory
   */
  public record Solution(
      double rpm,
      Rotation2d hoodPitch,
      Rotation2d fieldYaw,
      Rotation2d robotYaw,
      Rotation2d chassisYaw,
      boolean achievable) {}

  /** Configuration, built once at the start. */
  public static class Config {
    private InterpolatingDoubleTreeMap rpmMap;
    private InterpolatingDoubleTreeMap hoodAngleDegMap;
    private Translation2d shooterOffset = new Translation2d();
    private Rotation2d shooterFacing = new Rotation2d();
    private double wheelDiameterMeters = 0.0762; // 3 in
    private double efficiency = 0.9;
    private double minHoodDeg = 0.0;
    private double maxHoodDeg = 90.0;
    private double minRpm = 0.0;
    private double maxRpm = Double.POSITIVE_INFINITY;
    private ClampStrategy clampStrategy = ClampStrategy.LEAST_SQUARES;

    public Config rpmMap(InterpolatingDoubleTreeMap map) {
      this.rpmMap = map;
      return this;
    }

    public Config hoodAngleDegMap(InterpolatingDoubleTreeMap map) {
      this.hoodAngleDegMap = map;
      return this;
    }

    /** Shooter position relative to robot center, robot frame (x forward, y left), meters. */
    public Config shooterOffset(double xMeters, double yMeters) {
      this.shooterOffset = new Translation2d(xMeters, yMeters);
      return this;
    }

    /**
     * Direction a FIXED shooter fires relative to robot forward: 0 deg = out the front, 180 deg =
     * out the back. Independent of {@code shooterOffset}, which is only the shooter's position.
     */
    public Config shooterFacingDeg(double degrees) {
      this.shooterFacing = Rotation2d.fromDegrees(degrees);
      return this;
    }

    /** Flywheel diameter (both wheels assumed the same surface speed). */
    public Config wheelDiameterMeters(double meters) {
      this.wheelDiameterMeters = meters;
      return this;
    }

    /** Ball exit speed / wheel surface speed. Post suggests ~0.9. */
    public Config efficiency(double efficiency) {
      this.efficiency = efficiency;
      return this;
    }

    public Config hoodLimitsDeg(double minDeg, double maxDeg) {
      this.minHoodDeg = minDeg;
      this.maxHoodDeg = maxDeg;
      return this;
    }

    public Config rpmLimits(double minRpm, double maxRpm) {
      this.minRpm = minRpm;
      this.maxRpm = maxRpm;
      return this;
    }

    public Config clampStrategy(ClampStrategy strategy) {
      this.clampStrategy = strategy;
      return this;
    }
  }

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final Supplier<Translation2d> targetSupplier;

  private final InterpolatingDoubleTreeMap rpmMap;
  private final InterpolatingDoubleTreeMap hoodAngleDegMap;
  private final Translation2d shooterOffset;
  private final Rotation2d shooterFacing;
  private final double wheelCircumference;
  private final double efficiency;
  private final double minHoodRad;
  private final double maxHoodRad;
  private final double minRpm;
  private final double maxRpm;
  private final ClampStrategy clampStrategy;

  /**
   * @param poseSupplier field-relative robot pose
   * @param fieldSpeedsSupplier FIELD-relative chassis speeds
   * @param targetSupplier target position on the field (supplier so you can flip per alliance)
   * @param config solver configuration
   */
  public ShootOnTheMoveSolver(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      Supplier<Translation2d> targetSupplier,
      Config config) {
    if (config.rpmMap == null || config.hoodAngleDegMap == null) {
      throw new IllegalArgumentException("rpmMap and hoodAngleDegMap are required");
    }
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.targetSupplier = targetSupplier;
    this.rpmMap = config.rpmMap;
    this.hoodAngleDegMap = config.hoodAngleDegMap;
    this.shooterOffset = config.shooterOffset;
    this.shooterFacing = config.shooterFacing;
    this.wheelCircumference = Math.PI * config.wheelDiameterMeters;
    this.efficiency = config.efficiency;
    this.minHoodRad = Math.toRadians(config.minHoodDeg);
    this.maxHoodRad = Math.toRadians(config.maxHoodDeg);
    this.minRpm = config.minRpm;
    this.maxRpm = config.maxRpm;
    this.clampStrategy = config.clampStrategy;
  }

  /** Solve using the suppliers given in the constructor. Call every loop. */
  public Solution calculate() {
    return calculate(poseSupplier.get(), fieldSpeedsSupplier.get(), targetSupplier.get());
  }

  /** Pure version, same math with explicit inputs. */
  public Solution calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d target) {
    Rotation2d heading = robotPose.getRotation();

    // Shooter position and velocity in the field frame (includes rotation about robot center).
    Translation2d offsetField = shooterOffset.rotateBy(heading);
    Translation2d shooterPos = robotPose.getTranslation().plus(offsetField);
    double omega = fieldSpeeds.omegaRadiansPerSecond;
    double shooterVx = fieldSpeeds.vxMetersPerSecond - omega * offsetField.getY();
    double shooterVy = fieldSpeeds.vyMetersPerSecond + omega * offsetField.getX();

    // Static shot for this position.
    Translation2d toTarget = target.minus(shooterPos);
    double distance = toTarget.getNorm();
    double phiStar = Math.atan2(toTarget.getY(), toTarget.getX());
    double staticSpeed = rpmToSpeed(rpmMap.get(distance));
    double staticElev = Math.toRadians(hoodAngleDegMap.get(distance));
    double vHorizStar = staticSpeed * Math.cos(staticElev);
    double vZStar = staticSpeed * Math.sin(staticElev);

    // Velocity subtraction: v_shot = v_star - v_shooter (3D, robot has no vertical velocity).
    double reqX = vHorizStar * Math.cos(phiStar) - shooterVx;
    double reqY = vHorizStar * Math.sin(phiStar) - shooterVy;
    double reqZ = vZStar;
    double reqHoriz = Math.hypot(reqX, reqY);

    double fieldYawRad = reqHoriz < 1e-9 ? phiStar : Math.atan2(reqY, reqX);
    double reqElev = Math.atan2(reqZ, reqHoriz); // no division, safe near vertical

    boolean achievable = true;
    double elev = reqElev;
    double speed;

    if (reqElev < minHoodRad || reqElev > maxHoodRad) {
      achievable = false;
      elev = Math.max(minHoodRad, Math.min(maxHoodRad, reqElev));
      double cos = Math.cos(elev);
      double sin = Math.sin(elev);
      switch (clampStrategy) {
        case MATCH_HORIZONTAL:
          // Guard: hood pinned near vertical makes cos ~ 0, horizontal projection undefined.
          speed = Math.abs(cos) < 1e-6 ? Math.sqrt(reqHoriz * reqHoriz + reqZ * reqZ) : reqHoriz / cos;
          break;
        case LEAST_SQUARES:
        default:
          speed = Math.max(0.0, reqHoriz * cos + reqZ * sin);
          break;
      }
    } else {
      speed = Math.sqrt(reqX * reqX + reqY * reqY + reqZ * reqZ);
    }

    double rpm = speedToRpm(speed);
    if (rpm < minRpm || rpm > maxRpm) {
      achievable = false;
      rpm = Math.max(minRpm, Math.min(maxRpm, rpm));
    }

    Rotation2d fieldYaw = new Rotation2d(fieldYawRad);
    return new Solution(
        rpm,
        new Rotation2d(elev),
        fieldYaw,
        fieldYaw.minus(heading),
        fieldYaw.minus(shooterFacing),
        achievable);
  }

  private double rpmToSpeed(double rpm) {
    return (rpm / 60.0) * wheelCircumference * efficiency;
  }

  private double speedToRpm(double speedMps) {
    return speedMps / (wheelCircumference * efficiency) * 60.0;
  }
}