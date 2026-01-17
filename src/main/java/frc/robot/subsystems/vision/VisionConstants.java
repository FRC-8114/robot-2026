
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

public class VisionConstants {
    public static final EstimationMode LIMELIGHT_ESTIMATION_MODE = EstimationMode.MEGATAG2;

    public interface CameraConfiguration {
        String name();
        double stdDeviation();
    }

    public static class LimelightCameraConfiguration implements CameraConfiguration {
        private final String camera;
        private final double stdDeviation;

        public LimelightCameraConfiguration(String cameraName, double stdDeviation) {
            this.camera = cameraName;
            this.stdDeviation = stdDeviation;
        }

        @Override
        public double stdDeviation() {
            return stdDeviation;
        }

        @Override
        public String name() {
            return camera;
        }
    }

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Transforms are not used in limelights, these are used for photonvision sim
    public static final CameraConfiguration[] cameras = {
            new LimelightCameraConfiguration("camera_0", 1.0),
            new LimelightCameraConfiguration("camera_1", 1.0)
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