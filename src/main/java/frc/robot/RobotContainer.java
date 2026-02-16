// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.shooterpitch.ShooterPitchIOReal;
import frc.robot.subsystems.shooterpitch.ShooterPitchIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.simulation.GamePieceTracker;
import frc.robot.supersystems.ShooterSupersystem;
import frc.robot.util.FuelSim;

public class RobotContainer {
    private final Drive drive;
    private final Vision vision;
    private final Turret turretPivot;
    private final ShooterPitch turretPitch;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final ShooterSupersystem shooterSupersystem;
    private GamePieceTracker gamePieceTracker;

    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        switch (Constants.ROBOT_MODE) {
            case REAL: {
                drive = new Drive(new GyroIOPigeon2(), new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight), new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                vision = Vision.fromCameraConstants(poseEstimation -> {
                    drive.addVisionMeasurement(poseEstimation.pose().toPose2d(), poseEstimation.timestamp(),
                            poseEstimation.stddev());
                });

                turretPivot = new Turret(new TurretIOReal());
                turretPitch = new ShooterPitch(new ShooterPitchIOReal());
                indexer = new Indexer(new IndexerIOReal());
                intake = new Intake(new IntakeIOReal());
                shooter = new Shooter(new ShooterIOReal());
                break;
            }
            case SIMULATION: {
                drive = new Drive(new GyroIOPigeon2(), new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight), new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                var simVisionIOs = new ArrayList<frc.robot.subsystems.vision.VisionIO>();
                for (var cam : VisionConstants.cameras) {
                    simVisionIOs.add(new VisionIOPhotonVisionSim(cam, drive::getPose));
                }
                vision = new Vision(poseEstimation -> {
                    drive.addVisionMeasurement(poseEstimation.pose().toPose2d(), poseEstimation.timestamp(),
                            poseEstimation.stddev());
                }, simVisionIOs);

                turretPivot = new Turret(new TurretIOSim());
                turretPitch = new ShooterPitch(new ShooterPitchIOSim());
                indexer = new Indexer(new IndexerIOSim());
                intake = new Intake(new IntakeIOSim());
                shooter = new Shooter(new ShooterIOSim());

                // FuelSim setup
                var fuelSim = new FuelSim();
                fuelSim.registerRobot(
                        0.7, 0.7, 0.15,
                        drive::getPose,
                        () -> {
                            var cs = drive.getChassisSpeeds();
                            var heading = drive.getPose().getRotation();
                            return new ChassisSpeeds(
                                    cs.vxMetersPerSecond * heading.getCos()
                                            - cs.vyMetersPerSecond * heading.getSin(),
                                    cs.vxMetersPerSecond * heading.getSin()
                                            + cs.vyMetersPerSecond * heading.getCos(),
                                    cs.omegaRadiansPerSecond);
                        });
                fuelSim.spawnStartingFuel();
                fuelSim.start();

                gamePieceTracker = new GamePieceTracker(
                        fuelSim, indexer, shooter, turretPitch, turretPivot, drive);

                fuelSim.registerIntake(
                        0.2, 0.5, -0.2, 0.2,
                        () -> intake.getRollerRPMs() > 500,
                        () -> gamePieceTracker.onIntake());

                break;
            }
            case REPLAY: {
                throw new UnsupportedOperationException("No replay mode implemented");
            }
            default:
                throw new IllegalStateException("Unexpected value: " + Constants.ROBOT_MODE);
        }

        shooterSupersystem = new ShooterSupersystem(turretPivot, turretPitch, shooter, indexer);

        configureButtonBindings();
        setupAutoChoices();
    }

    private LoggedDashboardChooser<Command> autoChooser;

    private void setupAutoChoices() {
        autoChooser = new LoggedDashboardChooser<>("Auto Choices",
                AutoBuilder.buildAutoChooser());

        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Turret Pivot SysId
        autoChooser.addOption(
                "Turret Pivot SysId (Quasistatic Forward)",
                turretPivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Turret Pivot SysId (Quasistatic Reverse)",
                turretPivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Turret Pivot SysId (Dynamic Forward)",
                turretPivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Turret Pivot SysId (Dynamic Reverse)",
                turretPivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Turret Pitch SysId
        autoChooser.addOption(
                "Turret Pitch SysId (Quasistatic Forward)",
                turretPitch.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Turret Pitch SysId (Quasistatic Reverse)",
                turretPitch.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Turret Pitch SysId (Dynamic Forward)",
                turretPitch.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Turret Pitch SysId (Dynamic Reverse)",
                turretPitch.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Intake Deploy SysId
        autoChooser.addOption(
                "Intake Deploy SysId (Quasistatic Forward)",
                intake.deploySysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Intake Deploy SysId (Quasistatic Reverse)",
                intake.deploySysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Intake Deploy SysId (Dynamic Forward)",
                intake.deploySysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Intake Deploy SysId (Dynamic Reverse)",
                intake.deploySysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Intake Roller SysId
        autoChooser.addOption(
                "Intake Roller SysId (Quasistatic Forward)",
                intake.rollerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Intake Roller SysId (Quasistatic Reverse)",
                intake.rollerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Intake Roller SysId (Dynamic Forward)",
                intake.rollerSysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Intake Roller SysId (Dynamic Reverse)",
                intake.rollerSysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Shooter SysId
        autoChooser.addOption(
                "Shooter SysId (Quasistatic Forward)",
                shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Shooter SysId (Quasistatic Reverse)",
                shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Shooter SysId (Dynamic Forward)",
                shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Shooter SysId (Dynamic Reverse)",
                shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                                drive)
                                .ignoringDisable(true));

        controller.leftBumper().onTrue(intake.intake());
        controller.rightBumper().onTrue(intake.stow());
    }

    public void simulationPeriodic() {
        if (gamePieceTracker != null) {
            gamePieceTracker.updateSim();
        }
    }

    public GamePieceTracker getGamePieceTracker() {
        return gamePieceTracker;
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
