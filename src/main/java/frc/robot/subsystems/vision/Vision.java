// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class Vision {

  private final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public final double maxAmbiguity = 0.3;
  public final double maxZError = 0.75;
  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public final double linearStdDevBaseline = 0.02; // Meters
  public final double angularStdDevBaseline = 0.06; // Radians
  // Multipliers to apply for MegaTag 2 observations
  public final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve

  private final HootAutoReplay autoReplay = new HootAutoReplay();
  private final DriveTrainUpdater updater;
  private final VisionIO[] visionIOs;
  private VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;
  /** Creates a new Vision. */
  public Vision(CommandSwerveDrivetrain drivetrain,VisionIO... ios) {
    updater = drivetrain::addVisionMeasurement;
    visionIOs = ios;
    inputs = new VisionIOInputs[ios.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    this.disconnectedAlerts = new Alert[ios.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
    for (int index = 0; index < ios.length; index++) {
      int i = index;
      autoReplay.withStruct("VisionInputs/" + i, 
        new VisionIO.VisionIOInputs.VisionIOInputsStruct(
          ()-> inputs[i].poseObservations.size(), ()->inputs[i].tagIds.size()), 
          ()-> inputs[i], 
          (input)-> inputs[i] = input.value);
    }
  }
  /** call from robot periodic */
  public void periodic() {
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(inputs[i]);
    }
    autoReplay.update();
    for (int cameraIndex = 0; cameraIndex < visionIOs.length; cameraIndex++) {

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= Double.POSITIVE_INFINITY;
        }
          linearStdDev *= visionIOs[cameraIndex].getStdDevFactors();
          angularStdDev *= visionIOs[cameraIndex].getStdDevFactors();

        // Send vision observation
        updater.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }
  }

   /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }





















  @FunctionalInterface
  public static interface DriveTrainUpdater {
        public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
