package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;
import frc.robot.subsystems.vision.VisionConstants.LimelightCameraConfiguration;
import frc.robot.subsystems.vision.VisionIO.PoseEstimation;
import frc.robot.subsystems.vision.VisionIO.PoseEstimationBuffer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private record IOBundle(VisionIO io, VisionIOInputsAutoLogged inputs, PoseEstimationBuffer buffer) {}
    private final List<IOBundle> ioList;
    private final ArrayList<PoseEstimation> tagObservations = new ArrayList<PoseEstimation>();
    private final Consumer<PoseEstimation> poseConsumer;

    private final List<Pose3d> allRejectedPoses = new ArrayList<Pose3d>();
    private final List<Pose3d> allPoses = new ArrayList<Pose3d>();

    public Vision(Consumer<PoseEstimation> consumer, List<VisionIO> visionIOs) {
        ioList = visionIOs.stream()
                .map(io -> new IOBundle(io, new VisionIOInputsAutoLogged(), new PoseEstimationBuffer()))
                .toList();

        poseConsumer = consumer;
    }

    public static Vision fromCameraConstants(Consumer<PoseEstimation> consumer) {
        List<VisionIO> visionIOs = new ArrayList<>();

        for (CameraConfiguration camera : VisionConstants.cameras) {
            if (camera instanceof LimelightCameraConfiguration limelightCamera) {
                visionIOs.add(new VisionIOLimelight(limelightCamera));
            } else {
                throw new IllegalArgumentException("Unknown camera configuration type: " + camera.getClass());
            }
        }

        return new Vision(consumer, visionIOs);
    }

    private void processPoseEstimations() {
        if (tagObservations.size() == 0) {
            return;
        }

        for (PoseEstimation observation : tagObservations) {
            if (observation.ambiguity() > VisionConstants.maxAmbiguity) {
                allRejectedPoses.add(observation.pose());
                continue;
            }

            allPoses.add(observation.pose());
            poseConsumer.accept(observation);
        }
    }

    @Override
    public void periodic() {
        allPoses.clear();
        allRejectedPoses.clear();

        for (var bundle : ioList) {
            bundle.io().updateInputs(bundle.inputs(), bundle.buffer());
            Logger.processInputs("Vision/" + bundle.io().getConfiguration().name(), bundle.inputs());

            if (bundle.buffer().count > 0) {
                for (PoseEstimation est : bundle.buffer().poseEstimations) {
                    if (est != null) {
                        tagObservations.add(est);
                    }
                }
                bundle.buffer().clear();
            }
        }

        processPoseEstimations();
        tagObservations.clear();

        Logger.recordOutput("Vision/AllPoses", allPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/AllRejectedPoses", allRejectedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/AcceptedCount", allPoses.size());
        Logger.recordOutput("Vision/RejectedCount", allRejectedPoses.size());
    }

}