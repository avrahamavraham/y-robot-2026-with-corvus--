package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;

public interface VisionIO {
    public default void updateInputs(VisionIOInputs inputs) {}

    public double getStdDevFactors();

    public class VisionIOInputs {

        public boolean connected = false;
        public TargetObservation latestTargetObservation =
            new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public List<PoseObservation> poseObservations = new ArrayList<>();
        public List<Integer> tagIds = new ArrayList<>();

        public VisionIOInputs() {}

        public static class VisionIOInputsStruct implements Struct<VisionIOInputs> {
            // Struct arrays are fixed-size: anything past these limits is dropped when logging.
            public IntSupplier poseObservationsAmount = ()-> 8;
            public IntSupplier tagIsAmount = ()-> 16;
            private static final PoseObservation EMPTY_POSE_OBSERVATION =
                new PoseObservation(0.0, Pose3d.kZero, 0.0, 0, 0.0, PoseObservationType.MEGATAG_1);
            public VisionIOInputsStruct(IntSupplier poseObservationsAmount,IntSupplier tagIsAmount){
                this.poseObservationsAmount = poseObservationsAmount;
                this.tagIsAmount = tagIsAmount;
            }
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
                    + poseObservationsAmount.getAsInt() * PoseObservation.struct.getSize()
                    + kSizeInt8                                           // valid tagIds count
                    + tagIsAmount.getAsInt() * kSizeInt32;                           // tagIds
            }

            @Override
            public String getSchema() {
                return "bool connected;"
                    + "Rotation2d tx;"
                    + "Rotation2d ty;"
                    + "int8 poseCount;"
                    + "PoseObservation poses[" + poseObservationsAmount + "];"
                    + "int8 tagCount;"
                    + "int32 tagIds[" + tagIsAmount + "]";
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

                int poseCount = Math.min(bb.get(), poseObservationsAmount.getAsInt());
                inputs.poseObservations = new ArrayList<>(poseCount);
                // Always consume every slot so the buffer position stays correct
                for (int i = 0; i < poseObservationsAmount.getAsInt(); i++) {
                    PoseObservation obs = PoseObservation.struct.unpack(bb);
                    if (i < poseCount) {
                        inputs.poseObservations.add(obs);
                    }
                }

                int tagCount = Math.min(bb.get(), tagIsAmount.getAsInt());
                inputs.tagIds = new ArrayList<>(tagCount);
                for (int i = 0; i < tagIsAmount.getAsInt(); i++) {
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

                int poseCount = Math.min(value.poseObservations.size(), poseObservationsAmount.getAsInt());
                bb.put((byte) poseCount);
                // Always write every slot (padding with empties) so size is constant
                for (int i = 0; i < poseObservationsAmount.getAsInt(); i++) {
                    PoseObservation.struct.pack(
                        bb, i < poseCount ? value.poseObservations.get(i) : EMPTY_POSE_OBSERVATION);
                }

                int tagCount = Math.min(value.tagIds.size(), tagIsAmount.getAsInt());
                bb.put((byte) tagCount);
                for (int i = 0; i < tagIsAmount.getAsInt(); i++) {
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