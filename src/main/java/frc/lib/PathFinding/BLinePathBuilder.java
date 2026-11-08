package frc.lib.PathFinding;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import Path from your BLine package
import frc.robot.lib.BLine.Path;

public final class BLinePathBuilder {

  /** Elements to build the Path from, plus the ranged velocity caps. */
  public record Result(List<Path.PathElement> elements,
                       Path.RangedConstraint[] velocityConstraints) {
    public Path toPath() {
      Path.PathConstraints constraints =
          new Path.PathConstraints().setMaxVelocityMetersPerSec(velocityConstraints);
      return new Path(elements, constraints);
    }
  }

  private static final double SPEED_STEP = 0.25;                    // m/s, merges similar caps into ranges
  private static final double STRAIGHT_DEADBAND = Math.toRadians(8); // smaller turns count as straight

  /**
   * @param points     A* output, start to goal
   * @param startRot   heading at the first point
   * @param endRot     heading at the last point
   * @param vMax       speed on straight sections (m/s)
   * @param vCorner    speed at a 90 degree (or sharper) turn (m/s)
   * @param vFinal     speed cap when arriving at the final point (m/s)
   */
  public static Result build(List<Translation2d> points, Rotation2d startRot, Rotation2d endRot,
                             double vMax, double vCorner, double vFinal) {
    // 1. remove duplicate points (they make the turn angle undefined)
    List<Translation2d> pts = new ArrayList<>();
    for (Translation2d p : points) {
      if (pts.isEmpty() || pts.get(pts.size() - 1).getDistance(p) > 1e-3) pts.add(p);
    }
    int n = pts.size();
    if (n < 2) throw new IllegalArgumentException("need at least 2 distinct points");

    // 2. path elements: first and last are Waypoints (they carry a rotation), the rest are translation targets
    List<Path.PathElement> elements = new ArrayList<>();
    elements.add(new Path.Waypoint(pts.get(0), startRot));
    for (int i = 1; i < n - 1; i++) elements.add(new Path.TranslationTarget(pts.get(i)));
    elements.add(new Path.Waypoint(pts.get(n - 1), endRot));

    // 3. speed cap per ordinal, based on the turn angle at each intermediate point
    double[] cap = new double[n];
    java.util.Arrays.fill(cap, vMax);
    for (int i = 1; i < n - 1; i++) {
      Translation2d a = pts.get(i).minus(pts.get(i - 1));
      Translation2d b = pts.get(i + 1).minus(pts.get(i));
      double turn = Math.abs(MathUtil.angleModulus(b.getAngle().minus(a.getAngle()).getRadians()));
      if (turn < STRAIGHT_DEADBAND) continue;

      double t = MathUtil.clamp(turn / (Math.PI / 2), 0.0, 1.0);
      double v = vMax + (vCorner - vMax) * t;
      cap[i] = Math.min(cap[i], v);       // the turn point
      cap[i - 1] = Math.min(cap[i - 1], v); // and the point before it, so the robot is already slow
    }
    cap[n - 1] = Math.min(cap[n - 1], vFinal);

    // 4. quantize, then merge runs of equal caps into non-overlapping ranges
    for (int i = 0; i < n; i++) {
      cap[i] = Math.max(SPEED_STEP, Math.round(cap[i] / SPEED_STEP) * SPEED_STEP);
    }
    List<Path.RangedConstraint> ranges = new ArrayList<>();
    int start = 0;
    for (int i = 1; i <= n; i++) {
      if (i == n || cap[i] != cap[start]) {
        ranges.add(new Path.RangedConstraint(cap[start], start, i - 1)); // inclusive, zero-based
        start = i;
      }
    }

    return new Result(elements, ranges.toArray(new Path.RangedConstraint[0]));
  }
}