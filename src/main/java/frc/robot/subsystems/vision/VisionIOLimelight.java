package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;

import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;
import frc.robot.subsystems.vision.VisionConstants.LimelightCameraConfiguration;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class VisionIOLimelight implements VisionIO {
    public final Limelight limelight;
    public final LimelightPoseEstimator poseEstimator;
    private final LimelightPoseEstimator megatag1Estimator;
    private final LimelightCameraConfiguration config;

    public VisionIOLimelight(LimelightCameraConfiguration camera) {
        limelight = new Limelight(camera.name());
        poseEstimator = limelight.createPoseEstimator(VisionConstants.LIMELIGHT_ESTIMATION_MODE);
        megatag1Estimator = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
        config = camera;

        limelight.getSettings()
                .withPipelineIndex(0)
                .withCameraOffset(new Pose3d(
                        camera.robotToCamera().getTranslation(),
                        camera.robotToCamera().getRotation()))
                .withImuMode(LimelightSettings.ImuMode.InternalImuExternalAssist)
                .save();
    }

    public CameraConfiguration getConfiguration() {
        return config;
    }

    public void seedPoseFromMegatag1(Consumer<PoseEstimation> poseConsumer) {
        Optional<PoseEstimate> estimate = megatag1Estimator.getPoseEstimate();
        if (estimate.isEmpty()) {
            return;
        }

        PoseEstimate estimated = estimate.get();
        boolean willReject = estimated.tagCount == 0
                || (estimated.tagCount == 1 && estimated.getAvgTagAmbiguity() > maxAmbiguity)
                || estimated.pose.getX() < 0.0
                || estimated.pose.getX() > aprilTagLayout.getFieldLength()
                || estimated.pose.getY() < 0.0
                || estimated.pose.getY() > aprilTagLayout.getFieldWidth();

        if (!willReject) {
            poseConsumer.accept(new VisionIO.PoseEstimation(
                    estimated.pose,
                    estimated.timestampSeconds,
                    estimated.getAvgTagAmbiguity(),
                    new Matrix<>(VecBuilder.fill(1e-4, 1e-4, 1e-4))));
        }
    }

    public void setRobotOrientation(Rotation3d gyroRotation3d, Rotation3d gyroVelocityRadPerSec) {
        limelight.getSettings()
                .withRobotOrientation(new Orientation3d(
                        gyroRotation3d,
                        RadiansPerSecond.of(gyroVelocityRadPerSec.getX()),
                        RadiansPerSecond.of(gyroVelocityRadPerSec.getY()),
                        RadiansPerSecond.of(gyroVelocityRadPerSec.getZ())))
                .save();
    }

    public void updateInputs(VisionIOInputs inputs, PoseEstimationBuffer buffer) {
        inputs.connected = limelight.getNTTable().containsKey("getpipe");

        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();

        if (estimate.isEmpty()) {
            return;
        }

        PoseEstimate estimated = estimate.get();
        boolean willReject = estimated.tagCount == 0
                || (estimated.tagCount == 1
                        && estimated.getAvgTagAmbiguity() > maxAmbiguity)
                || Math.abs(estimated.pose.getZ()) > VisionConstants.maxZError
                || estimated.pose.getX() < 0.0
                || estimated.pose.getX() > aprilTagLayout.getFieldLength()
                || estimated.pose.getY() < 0.0
                || estimated.pose.getY() > aprilTagLayout.getFieldWidth();

        if (willReject) {
            Logger.recordOutput(
                    "Vision/Camera/" + config.name() + "/RejectedPose",
                    estimated.pose);
            return;
        }

        Logger.recordOutput(
                "Vision/Camera/" + config.name() + "/AcceptedPose",
                estimated.pose);
        buffer.pushEstimate(VisionIO.PoseEstimation.fromLimelightEstimate(this, estimated));
    }
}