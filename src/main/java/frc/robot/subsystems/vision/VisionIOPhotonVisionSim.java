package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim implements VisionIO {
    private static VisionSystemSim visionSim;
    private static double lastSimUpdateTimeSec = 0;

    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private final CameraConfiguration config;
    private final Supplier<Pose2d> poseSupplier;

    public VisionIOPhotonVisionSim(CameraConfiguration config, Supplier<Pose2d> poseSupplier) {
        this.config = config;
        this.poseSupplier = poseSupplier;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.aprilTagLayout);
        }

        camera = new PhotonCamera(config.name());

        var cameraProperties = SimCameraProperties.PERFECT_90DEG();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, config.robotToCamera());

        poseEstimator = new PhotonPoseEstimator(
                VisionConstants.aprilTagLayout,
                config.robotToCamera());
    }

    @Override
    public CameraConfiguration getConfiguration() {
        return config;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, PoseEstimationBuffer buffer) {
        double now = Timer.getFPGATimestamp();
        if (now - lastSimUpdateTimeSec > 0.005) {
            visionSim.update(poseSupplier.get());
            lastSimUpdateTimeSec = now;
        }
        inputs.connected = camera.isConnected();

        var results = camera.getAllUnreadResults();
        for (var result : results) {
            Optional<EstimatedRobotPose> estimated = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimated.isEmpty()) {
                estimated = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            if (estimated.isEmpty()) {
                continue;
            }

            EstimatedRobotPose pose = estimated.get();
            int tagCount = pose.targetsUsed.size();
            if (tagCount == 0) {
                continue;
            }

            double avgDist = pose.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

            double avgAmbiguity = pose.targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average()
                    .orElse(0.0);

            double stdDevFactor = Math.pow(avgDist, 2.0) / tagCount;
            double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor * config.stdDeviation();
            double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor * config.stdDeviation();

            Matrix<N3, N1> stddev = new Matrix<>(VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

            buffer.pushEstimate(new PoseEstimation(
                    pose.estimatedPose,
                    pose.timestampSeconds,
                    avgAmbiguity,
                    stddev));
        }
    }
}
