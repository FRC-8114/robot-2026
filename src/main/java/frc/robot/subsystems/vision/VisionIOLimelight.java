package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;
import frc.robot.subsystems.vision.VisionConstants.LimelightCameraConfiguration;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    public final Limelight limelight;
    public final LimelightPoseEstimator poseEstimator;
    private final LimelightCameraConfiguration config;

    public VisionIOLimelight(LimelightCameraConfiguration camera) {
        limelight = new Limelight(camera.name());
        poseEstimator = limelight.createPoseEstimator(VisionConstants.LIMELIGHT_ESTIMATION_MODE);
        config = camera;
    }

    @Override
    public CameraConfiguration getConfiguration() {
        return config;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();

        if (estimate.isEmpty()) {
            return;
        }

        PoseEstimate estimated = estimate.get();
        boolean willReject = estimated.tagCount == 0 // Must have at least one tag
                || (estimated.tagCount == 1
                        && estimated.getAvgTagAmbiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(estimated.pose.getZ()) > VisionConstants.maxZError
                // Must be within the field boundaries
                || estimated.pose.getX() < 0.0
                || estimated.pose.getX() > aprilTagLayout.getFieldLength()
                || estimated.pose.getY() < 0.0
                || estimated.pose.getY() > aprilTagLayout.getFieldWidth();

        if (willReject) {
            Logger.recordOutput(
                    "Vision/Camera" + config.name() + "/RejectedPose",
                    estimated.pose);
            return;
        }

        inputs.pushEstimate(VisionIO.PoseEstimation.fromLimelightEstimate(this, estimated));
    }
}