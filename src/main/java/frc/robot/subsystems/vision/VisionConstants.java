
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

public class VisionConstants {
    public static final EstimationMode LIMELIGHT_ESTIMATION_MODE = EstimationMode.MEGATAG2;

    public interface CameraConfiguration {
        String name();
        double stdDeviation();
        Transform3d robotToCamera();
    }

    public static class LimelightCameraConfiguration implements CameraConfiguration {
        private final String camera;
        private final double stdDeviation;
        private final Transform3d robotToCamera;

        public LimelightCameraConfiguration(String cameraName, double stdDeviation, Transform3d robotToCamera) {
            this.camera = cameraName;
            this.stdDeviation = stdDeviation;
            this.robotToCamera = robotToCamera;
        }

        @Override
        public double stdDeviation() {
            return stdDeviation;
        }

        @Override
        public String name() {
            return camera;
        }

        @Override
        public Transform3d robotToCamera() {
            return robotToCamera;
        }
    }

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // TODO: measure actual camera positions on robot
    private static final Transform3d ROBOT_TO_CAMERA_0 = new Transform3d(
            new Translation3d(0.3, 0.0, 0.25),
            new Rotation3d(0, Math.toRadians(-15), 0));

    private static final Transform3d ROBOT_TO_CAMERA_1 = new Transform3d(
            new Translation3d(-0.3, 0.0, 0.25),
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180)));

    public static final CameraConfiguration[] cameras = {
            new LimelightCameraConfiguration("camera_0", 1.0, ROBOT_TO_CAMERA_0),
            new LimelightCameraConfiguration("camera_1", 1.0, ROBOT_TO_CAMERA_1)
    };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}