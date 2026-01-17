// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    Drive drive;
    Vision vision;

    public RobotContainer() {
        switch (Constants.ROBOT_MODE) {
            case REAL:
                vision = Vision.fromCameraConstants(poseEstimation -> {
                    drive.addVisionMeasurement(poseEstimation.pose().toPose2d(), poseEstimation.timestamp(), null);
                });
                drive = new Drive(new GyroIOPigeon2(), new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight), new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
            case SIMULATION: {
                // TODO: sim
                
                // vision = new Vision(
                // drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(VisionConstants.camera0Name, robotToCamera0,
                // drive::getPose),
                // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

                break;
            }
            case REPLAY: {
                break;
            }
        }
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
