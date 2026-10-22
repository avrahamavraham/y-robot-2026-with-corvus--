package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    public class VisionIOInputs {
        public VisionIOInputs(){
            
        }
        /** One robot pose estimate computed from the camera. */
    public static record PoseObservation(
        /** Capture time in FPGA timebase, seconds. */
        double timestamp,
        /** Estimated robot pose in field (blue-origin) coordinates. */
        Pose3d pose,
        /** Tag pose ambiguity, 0 (good) to 1 (bad). Use 0 for multi-tag / MegaTag2. */
        double ambiguity,
        /** Number of tags used for this estimate. */
        int tagCount,
        /** Average distance from the camera to the tags used, meters. */
        double averageTagDistance,
        /** How this estimate was produced. */
        PoseObservationType type) {}
    
    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        SIM
    }
 

        
    }
}
