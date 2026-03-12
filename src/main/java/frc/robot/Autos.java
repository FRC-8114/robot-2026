package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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
    @FunctionalInterface
    private interface SlicedAutoBuilder {
        Command build(List<Command> pathCommands);
    }

    private IntakeRollers intakeRollers;
    private IntakePivot intakePivot;

    private ShooterSupersystem shooter;
    private Climber climber;

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

    private static ArrayList<Command> makeSlicedPathCommands(ArrayList<PathPlannerPath> paths) {
        ArrayList<Command> commands = new ArrayList<Command>();

        for (PathPlannerPath path : paths) {
            if (path == null) {
                commands.add(Commands.none());
            } else {
                commands.add(AutoBuilder.followPath(path));
            }
        }

        return commands;
    }

    private static Command resetPoseFromPath(PathPlannerPath path) {
        if (path == null) {
            return Commands.none();
        }

        return AutoBuilder.resetOdom(path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose()));
    }

    private static Command buildSlicedAuto(String choreoTraj, int numSlices, SlicedAutoBuilder builder) {
        var paths = loadSlicedPaths(choreoTraj, numSlices);
        var pathCommands = makeSlicedPathCommands(paths);

        return Commands.sequence(
            resetPoseFromPath(paths.isEmpty() ? null : paths.get(0)),
            builder.build(pathCommands)
        );
    }

    private Command shootSequence() {
        System.out.println("SHOOOOOTING!!!!!!!");
        return Commands.waitUntil(() -> false);
    }
    
    public Autos(IntakePivot intakePivot, IntakeRollers intakeRollers, Climber climber) {
        this.intakeRollers = intakeRollers;
        this.intakePivot = intakePivot;
        this.climber = climber;
    }

    public Command basicShoot() {
        return buildSlicedAuto("basicShoot", 3, pathSlices -> Commands.sequence(
            pathSlices.get(0),
            shootSequence()
                .withTimeout(Seconds.of(6)),
            pathSlices.get(1)
        ));
    }

    public Command trenchSSDepot() {
        return buildSlicedAuto("trenchSSDepot", 3, pathSlices -> Commands.sequence(
            intakePivot.deploy(),
            Commands.race( // collect balls from alliance zone
                intakeRollers.intake(),
                pathSlices.get(0)
            ),
            shootSequence()
                .withTimeout(Seconds.of(5)), // TODO: tune shoot time
            Commands.parallel( // collect balls from depot while shooting
                pathSlices.get(1), 
                shootSequence()
                    .withTimeout(Seconds.of(4)) // TODO: tune shoot time
            ),
            climber.deploy(),
            pathSlices.get(2), // approach climb
            climber.climb()
        ));
    }

    public Command trenchSSOutpost() {
        return buildSlicedAuto("trenchSSOutpost", 3, pathSlices -> Commands.sequence(
            intakePivot.deploy(),
            Commands.race( // collect balls from alliance zone
                intakeRollers.intake(),
                pathSlices.get(0)
            ),
            shootSequence()
                .withTimeout(Seconds.of(0)), // TODO: tune shoot time
            Commands.parallel( // collect balls from outpost while shooting
                pathSlices.get(1), 
                shootSequence()
                    .withTimeout(Seconds.of(0)) // TODO: tune shoot time
            ),
            climber.deploy(),
            pathSlices.get(2), // approach climb
            climber.climb()
        ));
    }
}
