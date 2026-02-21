package frc.robot.subsystems.vision;

import limelight.networktables.PoseEstimate;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;

public interface VisionIO {
    public static final int POSE_ESTIMATION_BUFFER_SIZE = 48;

    @AutoLog
    public static class VisionIOInputs {
        public boolean connected;
    }

    public static class PoseEstimationBuffer {
        public int count = 0;
        private int bufferPosition = 0;
        public PoseEstimation[] poseEstimations = new PoseEstimation[POSE_ESTIMATION_BUFFER_SIZE];

        public void pushEstimate(PoseEstimation estimation) {
            bufferPosition = (bufferPosition + 1) % POSE_ESTIMATION_BUFFER_SIZE;
            count = Math.min(count + 1, POSE_ESTIMATION_BUFFER_SIZE);

            poseEstimations[bufferPosition] = estimation;
        }

        public void clear() {
            count = 0;
            bufferPosition = 0;

            for (int i = 0; i < POSE_ESTIMATION_BUFFER_SIZE; i++) {
                poseEstimations[i] = null;
            }
        }
    }

    public static record PoseEstimation(
            Pose3d pose,
            double timestamp,
            double ambiguity,
            Matrix<N3, N1> stddev) {

        public static PoseEstimation fromLimelightEstimate(VisionIO visionIO, PoseEstimate estimate) {
            double stdDevFactor = Math.pow(estimate.avgTagDist, 2.0) / estimate.tagCount;
            double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
            double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

            if (estimate.isMegaTag2) {
                linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
                angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
            }

            linearStdDev *= visionIO.getConfiguration().stdDeviation();
            angularStdDev *= visionIO.getConfiguration().stdDeviation();

            return new PoseEstimation(
                    estimate.pose,
                    estimate.timestampSeconds,
                    estimate.getAvgTagAmbiguity(),
                    new Matrix<>(VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        }
    }

    CameraConfiguration getConfiguration();

    void updateInputs(VisionIOInputs inputs, PoseEstimationBuffer buffer);
}