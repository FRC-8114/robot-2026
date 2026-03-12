package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

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

    @Override
    public Optional<PoseEstimation> sampleSeedPose() {
        return megatag1Estimator.getPoseEstimate()
                .filter(estimate -> !shouldRejectEstimate(estimate, false))
                .map(VisionIO.PoseEstimation::seedFromLimelightEstimate);
    }

    private void setRobotOrientation(Rotation3d gyroRotation3d, Rotation3d gyroVelocityRadPerSec) {
        limelight.getSettings()
                .withRobotOrientation(new Orientation3d(
                        gyroRotation3d,
                        RadiansPerSecond.of(gyroVelocityRadPerSec.getZ()),
                        RadiansPerSecond.of(gyroVelocityRadPerSec.getY()),
                        RadiansPerSecond.of(gyroVelocityRadPerSec.getX())))
                .save();
    }

    private boolean shouldRejectEstimate(PoseEstimate estimate, boolean rejectOnZError) {
        return estimate.tagCount == 0
                || (estimate.tagCount == 1 && estimate.getAvgTagAmbiguity() > maxAmbiguity)
                || (rejectOnZError && Math.abs(estimate.pose.getZ()) > VisionConstants.maxZError)
                || estimate.pose.getX() < 0.0
                || estimate.pose.getX() > aprilTagLayout.getFieldLength()
                || estimate.pose.getY() < 0.0
                || estimate.pose.getY() > aprilTagLayout.getFieldWidth();
    }

    @Override
    public List<PoseEstimation> updateInputs(
            VisionIOInputs inputs,
            Rotation3d gyroRotation3d,
            Rotation3d gyroVelocityRadPerSec) {
        setRobotOrientation(gyroRotation3d, gyroVelocityRadPerSec);
        inputs.connected = limelight.getNTTable().containsKey("getpipe");

        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();
        if (estimate.isEmpty()) {
            return List.of();
        }

        PoseEstimate estimated = estimate.get();
        if (shouldRejectEstimate(estimated, true)) {
            Logger.recordOutput(
                    "Vision/Camera/" + config.name() + "/RejectedPose",
                    estimated.pose);
            return List.of();
        }

        Logger.recordOutput(
                "Vision/Camera/" + config.name() + "/AcceptedPose",
                estimated.pose);
        return List.of(VisionIO.PoseEstimation.fromLimelightEstimate(config, estimated));
    }

    @Override
    public void setIMUMode(LimelightSettings.ImuMode imuMode) {
        limelight.getSettings().withImuMode(imuMode).save();
    }
}
