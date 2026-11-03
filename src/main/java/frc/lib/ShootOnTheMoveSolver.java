package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Arrays;
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
 *   <li>Shot table, one row per tested distance: {@code shot(distanceMeters, rpm, hoodDeg)}. Hood is
 *       the launch elevation in degrees ABOVE HORIZONTAL (0 = flat, 90 = straight up). Rows may be
 *       added in any order; values between rows are linearly interpolated, outside the table the
 *       first/last row is used.
 *   <li>No adjustable hood: call {@code fixedHoodDeg(deg)} and add rows with {@code shot(distance,
 *       rpm)}. A turret is optional too: both {@code robotYaw} (turret target) and {@code
 *       chassisYaw} (fixed-shooter heading target) are always provided, use whichever you need.
 *   <li>Pose and speeds are FIELD-relative (CTRE swerve:
 *       {@code ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation())})
 *   <li>{@code shooterOffset}: shooter position in the robot frame (x forward, y left), meters
 * </ul>
 *
 * <p>Performance: after construction, {@code calculate} allocates nothing and reads only primitive
 * arrays. It therefore returns the SAME {@link Solution} object every call, overwritten each time:
 * read the values you need before the next call. Not thread-safe; call it from one thread.
 */
public class ShootOnTheMoveSolver {

  private static final double TWO_PI = 2.0 * Math.PI;

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
   * Result of a solve. One instance is reused and overwritten by every {@code calculate} call.
   *
   * <p>The {@code ...Rad()} getters are free. The {@code Rotation2d} getters create a new object on
   * each call, so prefer the {@code ...Rad()} ones inside the robot loop.
   */
  public static final class Solution {
    private double rpm;
    private double hoodPitchRad;
    private double fieldYawRad;
    private double robotYawRad;
    private double chassisYawRad;

    /** Flywheel speed in RPM. */
    public double rpm() {
      return rpm;
    }

    /** Launch elevation above horizontal, radians. */
    public double hoodPitchRad() {
      return hoodPitchRad;
    }

    /** Direction the SHOOTER must point, FIELD frame, radians in (-pi, pi]. */
    public double fieldYawRad() {
      return fieldYawRad;
    }

    /**
     * Direction the shooter must point relative to the robot's forward axis (turret target, before
     * any turret mounting offset), radians in (-pi, pi].
     */
    public double robotYawRad() {
      return robotYawRad;
    }

    /**
     * FIELD-frame heading the CHASSIS must have so a fixed shooter (facing {@code shooterFacing}
     * relative to robot forward) points along {@code fieldYaw}; use this as the target for
     * fixed-shooter heading control. Radians in (-pi, pi].
     */
    public double chassisYawRad() {
      return chassisYawRad;
    }

    /** Allocating convenience version of {@link #hoodPitchRad()}. */
    public Rotation2d hoodPitch() {
      return new Rotation2d(hoodPitchRad);
    }

    /** Allocating convenience version of {@link #fieldYawRad()}. */
    public Rotation2d fieldYaw() {
      return new Rotation2d(fieldYawRad);
    }

    /** Allocating convenience version of {@link #robotYawRad()}. */
    public Rotation2d robotYaw() {
      return new Rotation2d(robotYawRad);
    }

    /** Allocating convenience version of {@link #chassisYawRad()}. */
    public Rotation2d chassisYaw() {
      return new Rotation2d(chassisYawRad);
    }
  }

  /** Configuration, built once at the start. */
  public static class Config {
    // Shot table, kept sorted by distance. Hood NaN = "use fixedHoodDeg".
    private double[] distM = new double[8];
    private double[] rpms = new double[8];
    private double[] hoodDegs = new double[8];
    private int count = 0;

    private double offsetX = 0.0;
    private double offsetY = 0.0;
    private double shooterFacingDeg = 0.0;
    private double wheelDiameterMeters = 0.0762; // 3 in
    private double efficiency = 0.9;
    private double minHoodDeg = 0.0;
    private double maxHoodDeg = 90.0;
    private double fixedHoodDeg = Double.NaN;
    private double minRpm = 0.0;
    private double maxRpm = Double.POSITIVE_INFINITY;
    private ClampStrategy clampStrategy = ClampStrategy.LEAST_SQUARES;

    /**
     * Add one tested shot: at {@code distanceMeters} (shooter to target) the static shot is {@code
     * rpm} with the hood at {@code hoodDeg} above horizontal. Adding the same distance twice keeps
     * the last row.
     */
    public Config shot(double distanceMeters, double rpm, double hoodDeg) {
      return addShot(distanceMeters, rpm, hoodDeg);
    }

    /** Same as above for a robot with no adjustable hood. Needs {@link #fixedHoodDeg}. */
    public Config shot(double distanceMeters, double rpm) {
      return addShot(distanceMeters, rpm, Double.NaN);
    }

    /**
     * Robot without an adjustable hood: the launch angle is always {@code degrees} above
     * horizontal. Overrides {@code hoodLimitsDeg}, whatever order they are called in.
     */
    public Config fixedHoodDeg(double degrees) {
      this.fixedHoodDeg = degrees;
      return this;
    }

    /** Shooter position relative to robot center, robot frame (x forward, y left), meters. */
    public Config shooterOffset(double xMeters, double yMeters) {
      this.offsetX = xMeters;
      this.offsetY = yMeters;
      return this;
    }

    /**
     * Direction a FIXED shooter fires relative to robot forward: 0 deg = out the front, 180 deg =
     * out the back. Independent of {@code shooterOffset}, which is only the shooter's position.
     */
    public Config shooterFacingDeg(double degrees) {
      this.shooterFacingDeg = degrees;
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

    private Config addShot(double d, double rpm, double hoodDeg) {
      int i = 0;
      while (i < count && distM[i] < d) {
        i++;
      }
      if (i < count && distM[i] == d) {
        rpms[i] = rpm;
        hoodDegs[i] = hoodDeg;
        return this;
      }
      if (count == distM.length) {
        int cap = count * 2;
        distM = Arrays.copyOf(distM, cap);
        rpms = Arrays.copyOf(rpms, cap);
        hoodDegs = Arrays.copyOf(hoodDegs, cap);
      }
      int tail = count - i;
      System.arraycopy(distM, i, distM, i + 1, tail);
      System.arraycopy(rpms, i, rpms, i + 1, tail);
      System.arraycopy(hoodDegs, i, hoodDegs, i + 1, tail);
      distM[i] = d;
      rpms[i] = rpm;
      hoodDegs[i] = hoodDeg;
      count++;
      return this;
    }
  }

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final Supplier<Translation2d> targetSupplier;

  // Shot table, baked to the units the math uses: meters, m/s of ball speed, radians.
  private final double[] dist;
  private final double[] speed;
  private final double[] elev; // null when every row has the same hood angle
  private final boolean constElev;
  private final double constCos;
  private final double constSin;

  private final double offsetX;
  private final double offsetY;
  private final double shooterFacingRad;
  private final double rpmPerSpeed;
  private final double minHoodRad;
  private final double maxHoodRad;
  private final double minCos;
  private final double minSin;
  private final double maxCos;
  private final double maxSin;
  private final boolean hoodPinned; // min == max: no hood, or hood locked
  private final double minRpm;
  private final double maxRpm;
  private final boolean matchHorizontal;

  // Scratch + result, reused every call so the loop never allocates.
  private double lookSpeed;
  private double lookElev;
  private final Solution solution = new Solution();

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
    if (config.count == 0) {
      throw new IllegalArgumentException("add at least one shot(...) row to the Config");
    }
    boolean noHood = !Double.isNaN(config.fixedHoodDeg);
    double minHoodDeg = noHood ? config.fixedHoodDeg : config.minHoodDeg;
    double maxHoodDeg = noHood ? config.fixedHoodDeg : config.maxHoodDeg;
    if (minHoodDeg > maxHoodDeg) {
      throw new IllegalArgumentException("hood min limit is above the max limit");
    }
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.targetSupplier = targetSupplier;

    double speedPerRpm = Math.PI * config.wheelDiameterMeters * config.efficiency / 60.0;
    int n = config.count;
    double[] elevRad = new double[n];
    boolean same = true;
    this.dist = Arrays.copyOf(config.distM, n);
    this.speed = new double[n];
    for (int i = 0; i < n; i++) {
      speed[i] = config.rpms[i] * speedPerRpm;
      double hoodDeg = config.hoodDegs[i];
      if (Double.isNaN(hoodDeg)) {
        if (Double.isNaN(config.fixedHoodDeg)) {
          throw new IllegalArgumentException("shot(distance, rpm) rows need fixedHoodDeg(...)");
        }
        hoodDeg = config.fixedHoodDeg;
      }
      elevRad[i] = Math.toRadians(hoodDeg);
      if (elevRad[i] != elevRad[0]) {
        same = false;
      }
    }
    this.constElev = same;
    this.elev = same ? null : elevRad;
    this.constCos = Math.cos(elevRad[0]);
    this.constSin = Math.sin(elevRad[0]);

    this.offsetX = config.offsetX;
    this.offsetY = config.offsetY;
    this.shooterFacingRad = Math.toRadians(config.shooterFacingDeg);
    this.rpmPerSpeed = 1.0 / speedPerRpm;
    this.minHoodRad = Math.toRadians(minHoodDeg);
    this.maxHoodRad = Math.toRadians(maxHoodDeg);
    this.minCos = Math.cos(minHoodRad);
    this.minSin = Math.sin(minHoodRad);
    this.maxCos = Math.cos(maxHoodRad);
    this.maxSin = Math.sin(maxHoodRad);
    this.hoodPinned = minHoodRad == maxHoodRad;
    this.minRpm = config.minRpm;
    this.maxRpm = config.maxRpm;
    this.matchHorizontal = config.clampStrategy == ClampStrategy.MATCH_HORIZONTAL;
  }

  /** For callers that use the primitive {@code calculate} only; {@code calculate()} is unavailable. */
  public ShootOnTheMoveSolver(Config config) {
    this(null, null, null, config);
  }

  /** Solve using the suppliers given in the constructor. Call every loop while shooting. */
  public Solution calculate() {
    if (poseSupplier == null) {
      throw new IllegalStateException("built without suppliers, use calculate(pose, speeds, target)");
    }
    return calculate(poseSupplier.get(), fieldSpeedsSupplier.get(), targetSupplier.get());
  }

  /** Same math with explicit inputs. */
  public Solution calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d target) {
    Rotation2d heading = robotPose.getRotation();
    return solve(
        robotPose.getX(),
        robotPose.getY(),
        heading.getCos(),
        heading.getSin(),
        heading.getRadians(),
        fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond,
        fieldSpeeds.omegaRadiansPerSecond,
        target.getX(),
        target.getY());
  }

  /** Same math with plain numbers, so the caller does not have to build any objects either. */
  public Solution calculate(
      double robotXMeters,
      double robotYMeters,
      double headingRad,
      double fieldVxMps,
      double fieldVyMps,
      double omegaRadPerSec,
      double targetXMeters,
      double targetYMeters) {
    return solve(
        robotXMeters,
        robotYMeters,
        Math.cos(headingRad),
        Math.sin(headingRad),
        headingRad,
        fieldVxMps,
        fieldVyMps,
        omegaRadPerSec,
        targetXMeters,
        targetYMeters);
  }

  private Solution solve(
      double x,
      double y,
      double cosH,
      double sinH,
      double headingRad,
      double vx,
      double vy,
      double omega,
      double tx,
      double ty) {
    // Shooter position and velocity in the field frame (includes rotation about robot center).
    double offFieldX = offsetX * cosH - offsetY * sinH;
    double offFieldY = offsetX * sinH + offsetY * cosH;
    double dx = tx - (x + offFieldX);
    double dy = ty - (y + offFieldY);
    double shooterVx = vx - omega * offFieldY;
    double shooterVy = vy + omega * offFieldX;

    // Static shot for this distance. One lookup gives both ball speed and elevation.
    double distance = Math.sqrt(dx * dx + dy * dy);
    lookup(distance);
    double cosStar;
    double sinStar;
    if (constElev) {
      cosStar = constCos;
      sinStar = constSin;
    } else {
      cosStar = Math.cos(lookElev);
      sinStar = Math.sin(lookElev);
    }
    double vHorizStar = lookSpeed * cosStar;
    double vZStar = lookSpeed * sinStar;

    // Velocity subtraction: v_shot = v_star - v_shooter (3D, robot has no vertical velocity).
    // The static shot points along (dx, dy), so cos/sin of its direction are dx/distance and
    // dy/distance: no atan2/cos/sin needed.
    double reqX;
    double reqY;
    if (distance > 0.0) {
      double k = vHorizStar / distance;
      reqX = k * dx - shooterVx;
      reqY = k * dy - shooterVy;
    } else {
      reqX = vHorizStar - shooterVx;
      reqY = -shooterVy;
    }
    double reqZ = vZStar;
    double horizSq = reqX * reqX + reqY * reqY;
    double reqHoriz = Math.sqrt(horizSq); // sqrt, not hypot: hypot is far slower in Java

    double fieldYaw = reqHoriz < 1e-9 ? Math.atan2(dy, dx) : Math.atan2(reqY, reqX);

    // Hood: use the exact elevation if the hood can reach it, otherwise pin it to the nearest limit.
    // Trig of the limits is precomputed, so the clamped case needs no sin/cos either.
    double elevation;
    double cos = 0.0;
    double sin = 0.0;
    boolean clamped = true;
    if (hoodPinned) {
      elevation = minHoodRad;
      cos = minCos;
      sin = minSin;
    } else {
      elevation = Math.atan2(reqZ, reqHoriz); // no division, safe near vertical
      if (elevation < minHoodRad) {
        elevation = minHoodRad;
        cos = minCos;
        sin = minSin;
      } else if (elevation > maxHoodRad) {
        elevation = maxHoodRad;
        cos = maxCos;
        sin = maxSin;
      } else {
        clamped = false;
      }
    }

    double ballSpeed;
    if (!clamped) {
      ballSpeed = Math.sqrt(horizSq + reqZ * reqZ);
    } else if (matchHorizontal) {
      // Guard: hood pinned near vertical makes cos ~ 0, horizontal projection undefined.
      ballSpeed =
          Math.abs(cos) < 1e-6 ? Math.sqrt(horizSq + reqZ * reqZ) : reqHoriz / cos;
    } else {
      ballSpeed = Math.max(0.0, reqHoriz * cos + reqZ * sin);
    }

    double rpm = ballSpeed * rpmPerSpeed;
    rpm = Math.max(minRpm, Math.min(maxRpm, rpm));

    Solution s = solution;
    s.rpm = rpm;
    s.hoodPitchRad = elevation;
    s.fieldYawRad = fieldYaw;
    s.robotYawRad = wrapPi(fieldYaw - headingRad);
    s.chassisYawRad = wrapPi(fieldYaw - shooterFacingRad);
    return s;
  }

  /** Linear interpolation in the baked table, same behavior as InterpolatingDoubleTreeMap. */
  private void lookup(double d) {
    double[] xs = dist;
    int last = xs.length - 1;
    if (d <= xs[0]) {
      lookSpeed = speed[0];
      if (elev != null) {
        lookElev = elev[0];
      }
      return;
    }
    if (d >= xs[last]) {
      lookSpeed = speed[last];
      if (elev != null) {
        lookElev = elev[last];
      }
      return;
    }
    int lo = 0;
    int hi = last;
    while (hi - lo > 1) {
      int mid = (lo + hi) >>> 1;
      if (xs[mid] <= d) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    double t = (d - xs[lo]) / (xs[hi] - xs[lo]);
    lookSpeed = speed[lo] + (speed[hi] - speed[lo]) * t;
    if (elev != null) {
      lookElev = elev[lo] + (elev[hi] - elev[lo]) * t;
    }
  }

  /** Wrap to (-pi, pi]. Inputs are almost always already close, so the slow path is rare. */
  private static double wrapPi(double a) {
    if (a > Math.PI || a <= -Math.PI) {
      return Math.IEEEremainder(a, TWO_PI);
    }
    return a;
  }
}