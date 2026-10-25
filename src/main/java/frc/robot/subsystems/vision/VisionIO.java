package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;

public interface VisionIO {
    public default void updateInputs(VisionIOInputs inputs) {}

    public double getStdDevFactors();

    public class VisionIOInputs {
        // Struct arrays are fixed-size: anything past these limits is dropped when logging.
        public static final int MAX_POSE_OBSERVATIONS = 8;
        public static final int MAX_TAG_IDS = 16;

        public static final VisionIOInputsStruct struct = new VisionIOInputsStruct();

        public boolean connected = false;
        public TargetObservation latestTargetObservation =
            new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public List<PoseObservation> poseObservations = new ArrayList<>();
        public List<Integer> tagIds = new ArrayList<>();

        public VisionIOInputs() {}

        public static class VisionIOInputsStruct implements Struct<VisionIOInputs> {
            private static final PoseObservation EMPTY_POSE_OBSERVATION =
                new PoseObservation(0.0, Pose3d.kZero, 0.0, 0, 0.0, PoseObservationType.MEGATAG_1);

            @Override
            public Class<VisionIOInputs> getTypeClass() {
                return VisionIOInputs.class;
            }

            @Override
            public String getTypeName() {
                return "VisionIOInputs";
            }

            @Override
            public int getSize() {
                return kSizeBool                                          // connected
                    + Rotation2d.struct.getSize()                         // latestTargetObservation.tx
                    + Rotation2d.struct.getSize()                         // latestTargetObservation.ty
                    + kSizeInt8                                           // valid poseObservations count
                    + MAX_POSE_OBSERVATIONS * PoseObservation.struct.getSize()
                    + kSizeInt8                                           // valid tagIds count
                    + MAX_TAG_IDS * kSizeInt32;                           // tagIds
            }

            @Override
            public String getSchema() {
                return "bool connected;"
                    + "Rotation2d tx;"
                    + "Rotation2d ty;"
                    + "int8 poseCount;"
                    + "PoseObservation poses[" + MAX_POSE_OBSERVATIONS + "];"
                    + "int8 tagCount;"
                    + "int32 tagIds[" + MAX_TAG_IDS + "]";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] {Rotation2d.struct, PoseObservation.struct};
            }

            @Override
            public VisionIOInputs unpack(ByteBuffer bb) {
                VisionIOInputs inputs = new VisionIOInputs();

                inputs.connected = bb.get() != 0;

                Rotation2d tx = Rotation2d.struct.unpack(bb);
                Rotation2d ty = Rotation2d.struct.unpack(bb);
                inputs.latestTargetObservation = new TargetObservation(tx, ty);

                int poseCount = Math.min(bb.get(), MAX_POSE_OBSERVATIONS);
                inputs.poseObservations = new ArrayList<>(poseCount);
                // Always consume every slot so the buffer position stays correct
                for (int i = 0; i < MAX_POSE_OBSERVATIONS; i++) {
                    PoseObservation obs = PoseObservation.struct.unpack(bb);
                    if (i < poseCount) {
                        inputs.poseObservations.add(obs);
                    }
                }

                int tagCount = Math.min(bb.get(), MAX_TAG_IDS);
                inputs.tagIds = new ArrayList<>(tagCount);
                for (int i = 0; i < MAX_TAG_IDS; i++) {
                    int id = bb.getInt();
                    if (i < tagCount) {
                        inputs.tagIds.add(id);
                    }
                }

                return inputs;
            }

            @Override
            public void pack(ByteBuffer bb, VisionIOInputs value) {
                bb.put((byte) (value.connected ? 1 : 0));

                Rotation2d.struct.pack(bb, value.latestTargetObservation.tx());
                Rotation2d.struct.pack(bb, value.latestTargetObservation.ty());

                int poseCount = Math.min(value.poseObservations.size(), MAX_POSE_OBSERVATIONS);
                bb.put((byte) poseCount);
                // Always write every slot (padding with empties) so size is constant
                for (int i = 0; i < MAX_POSE_OBSERVATIONS; i++) {
                    PoseObservation.struct.pack(
                        bb, i < poseCount ? value.poseObservations.get(i) : EMPTY_POSE_OBSERVATION);
                }

                int tagCount = Math.min(value.tagIds.size(), MAX_TAG_IDS);
                bb.put((byte) tagCount);
                for (int i = 0; i < MAX_TAG_IDS; i++) {
                    bb.putInt(i < tagCount ? value.tagIds.get(i) : 0);
                }
            }

            @Override
            public boolean isImmutable() {
                return false;
            }
        }
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type) {

        public static final PoseObservationStruct struct = new PoseObservationStruct();
    }

    public static class PoseObservationStruct implements Struct<PoseObservation> {
        @Override
        public Class<PoseObservation> getTypeClass() {
            return PoseObservation.class;
        }

        @Override
        public String getTypeName() {
            return "PoseObservation";
        }

        @Override
        public int getSize() {
            return kSizeDouble              // timestamp
                + Pose3d.struct.getSize()   // pose
                + kSizeDouble               // ambiguity
                + kSizeInt32                // tagCount
                + kSizeDouble               // averageTagDistance
                + kSizeInt8;                // type (ordinal)
        }

        @Override
        public String getSchema() {
            return "double timestamp;"
                + "Pose3d pose;"
                + "double ambiguity;"
                + "int32 tagCount;"
                + "double averageTagDistance;"
                + "enum {MEGATAG_1=0, MEGATAG_2=1, PHOTONVISION=2} int8 type";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose3d.struct};
        }

        @Override
        public PoseObservation unpack(ByteBuffer bb) {
            double timestamp = bb.getDouble();
            Pose3d pose = Pose3d.struct.unpack(bb);
            double ambiguity = bb.getDouble();
            int tagCount = bb.getInt();
            double averageTagDistance = bb.getDouble();
            int typeOrdinal = bb.get();

            PoseObservationType[] types = PoseObservationType.values();
            PoseObservationType type =
                (typeOrdinal >= 0 && typeOrdinal < types.length) ? types[typeOrdinal] : types[0];

            return new PoseObservation(timestamp, pose, ambiguity, tagCount, averageTagDistance, type);
        }

        @Override
        public void pack(ByteBuffer bb, PoseObservation value) {
            bb.putDouble(value.timestamp());
            Pose3d.struct.pack(bb, value.pose());
            bb.putDouble(value.ambiguity());
            bb.putInt(value.tagCount());
            bb.putDouble(value.averageTagDistance());
            bb.put((byte) value.type().ordinal());
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}
}