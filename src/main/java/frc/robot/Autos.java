package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.supersystems.ShooterSupersystem;

public class Autos {
    private IntakeRollers intakeRollers;
    private IntakePivot intakePivot;

    private ShooterSupersystem shooter;
    private Climber climber;

    public Autos(IntakePivot intakePivot, IntakeRollers intakeRollers, Climber climber) {
        this.intakeRollers = intakeRollers;
        this.intakePivot = intakePivot;
        this.climber = climber;
    }

    private static ArrayList<PathPlannerPath> loadSlicedPaths(String choreoTraj, Integer num_slices) {
        ArrayList<PathPlannerPath> slices = new ArrayList<PathPlannerPath>();

        for (int i = 0; i < num_slices; i++) {
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreoTraj, i);
                if (path != null) {
                    slices.add(path);
                } else {
                    System.err.println("Warning: Path slice " + i + " for trajectory " + choreoTraj + " is null");
                    slices.add(null);
                }
            } catch (FileVersionException | IOException | org.json.simple.parser.ParseException e) {
                System.err.println("Error loading path slice " + i + " for trajectory " + choreoTraj);
                e.printStackTrace();
                slices.add(null);
            }
        }

        return slices;
    }

    private static Command resetOdomFromPath(PathPlannerPath path) {
        return AutoBuilder.resetOdom(
            path.getStartingHolonomicPose()
                .orElse(path.getStartingDifferentialPose())
        );
    }

    private Command shootSequence() {
        return Commands.waitUntil(() -> {
            System.out.println("SHOOOOOTING!!!!!!!");
            return false;
        });
    }

    public Command basicShoot() {
        var pathSlices = loadSlicedPaths("basicShoot", 3);

        return Commands.sequence(
            resetOdomFromPath(pathSlices.get(0)),
            AutoBuilder.followPath(pathSlices.get(0)),
            shootSequence().withTimeout(Seconds.of(6)),
            AutoBuilder.followPath(pathSlices.get(1)),
            climber.deploy(),
            AutoBuilder.followPath(pathSlices.get(2)),
            climber.climb()
        );
    }

    public Command TUNE_MOI() {
        var pathSlices = loadSlicedPaths("CalibrateMOI", 0);

        return Commands.sequence(
            resetOdomFromPath(pathSlices.get(0)),
            AutoBuilder.followPath(pathSlices.get(0))
        );
    }

    public Command trenchSSOutpost() {
        var pathSlices = loadSlicedPaths("trenchSSOutpost", 7);

        return Commands.sequence(
            resetOdomFromPath(pathSlices.get(0)),
            intakePivot.deploy(),
            Commands.race( // intake balls from alliance zone
                intakeRollers.intake(), // (stops when first path slice finishes)
                AutoBuilder.followPath(pathSlices.get(0))
            ),
            AutoBuilder.followPath(pathSlices.get(1)), // drive to shoot pose
            shootSequence() // shoot
                .withTimeout(Seconds.of(5)),
            AutoBuilder.followPath(pathSlices.get(2)), // go to outpost
            Commands.waitTime(Seconds.of(4)), // balls drop
            AutoBuilder.followPath(pathSlices.get(4)), // go to shoot pose
            shootSequence() // shoot
                .withTimeout(Seconds.of(5)),
            AutoBuilder.followPath(pathSlices.get(5)), // go to prepare climb
            climber.deploy(),
            AutoBuilder.followPath(pathSlices.get(6))
        );
    }

    // public Command trenchSSDepot() {
    //     return buildSlicedAuto("trenchSSDepot", 3, pathSlices -> Commands.sequence(
    //             intakePivot.deploy(),
    //             Commands.race( // collect balls from alliance zone
    //                     intakeRollers.intake(),
    //                     pathSlices.get(0)),
    //             shootSequence()
    //                     .withTimeout(Seconds.of(5)), // TODO: tune shoot time
    //             Commands.parallel( // collect balls from depot while shooting
    //                     pathSlices.get(1),
    //                     shootSequence()
    //                             .withTimeout(Seconds.of(4)) // TODO: tune shoot time
    //             ),
    //             climber.deploy(),
    //             pathSlices.get(2), // approach climb
    //             climber.climb()));
    // }

}
