package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import limelight.networktables.LimelightSettings;
import limelight.networktables.PoseEstimate;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected;
    }

    public static record PoseEstimation(
            Pose3d pose,
            double timestamp,
            double ambiguity,
            Matrix<N3, N1> stddev) {

        public static PoseEstimation seedFromLimelightEstimate(PoseEstimate estimate) {
            return new PoseEstimation(
                    estimate.pose,
                    estimate.timestampSeconds,
                    estimate.getAvgTagAmbiguity(),
                    new Matrix<>(VecBuilder.fill(1e-4, 1e-4, 1e-4)));
        }

        public static PoseEstimation fromLimelightEstimate(CameraConfiguration configuration, PoseEstimate estimate) {
            return fromStatistics(
                    configuration,
                    estimate.pose,
                    estimate.timestampSeconds,
                    estimate.getAvgTagAmbiguity(),
                    estimate.avgTagDist,
                    estimate.tagCount,
                    estimate.isMegaTag2);
        }

        public static PoseEstimation fromStatistics(
                CameraConfiguration configuration,
                Pose3d pose,
                double timestamp,
                double ambiguity,
                double avgTagDist,
                int tagCount,
                boolean isMegaTag2) {
            double stdDevFactor = Math.pow(avgTagDist, 2.0) / tagCount;
            double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
            double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

            if (isMegaTag2) {
                linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
                angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
            }

            linearStdDev *= configuration.stdDeviation();
            angularStdDev *= configuration.stdDeviation();

            return new PoseEstimation(
                    pose,
                    timestamp,
                    ambiguity,
                    new Matrix<>(VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        }
    }

    CameraConfiguration getConfiguration();

    default void setIMUMode(LimelightSettings.ImuMode imuMode) {};

    Optional<PoseEstimation> sampleSeedPose();

    List<PoseEstimation> updateInputs(
            VisionIOInputs inputs,
            Rotation3d gyroRotation3d,
            Rotation3d gyroVelocityRadPerSec);
}
