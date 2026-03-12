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

    private static ArrayList<Command> makeSlicedPath(String choreoTraj, Integer num_slices) {
        ArrayList<Command> slices = new ArrayList<Command>();

        for (int i = 0; i < num_slices; i++) {
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreoTraj, i);
                if (path != null) {
                    slices.add(AutoBuilder.followPath(path));
                } else {
                    System.err.println("Warning: Path slice " + i + " for trajectory " + choreoTraj + " is null");
                    slices.add(Commands.none());
                }
            } catch (FileVersionException | IOException | org.json.simple.parser.ParseException e) {
                System.err.println("Error loading path slice " + i + " for trajectory " + choreoTraj);
                e.printStackTrace();
                slices.add(Commands.none());
            }
        }

        return slices;
    }

    private Command shootSequence() {
        return Commands.waitUntil(() -> {
            System.out.println("SHOOOOOTING!!!!!!!");
            return false;
        });
    }
    
    public Autos(IntakePivot intakePivot, IntakeRollers intakeRollers, Climber climber) {
        this.intakeRollers = intakeRollers;
        this.intakePivot = intakePivot;
        this.climber = climber;
    }

    public Command basicShoot() {
        var pathSlices = makeSlicedPath("basicShoot", 3);

        return Commands.sequence(
            pathSlices.get(0),
            shootSequence()
                .withTimeout(Seconds.of(6)),
            pathSlices.get(1)
        );
    }

    public Command TUNE_MOI() {
        var pathSlices = makeSlicedPath("CalibrateMOI", 1);
        return pathSlices.get(0);
    }

    public Command trenchSSOutpost() {
        var pathSlices = makeSlicedPath("trenchSSOutpost", 6);

        return Commands.sequence(
            intakePivot.deploy(),
            Commands.race( // collect balls from alliance zone
                intakeRollers.intake(),
                pathSlices.get(0)
            ),
            pathSlices.get(1), // drive to shot
            shootSequence()
                .withTimeout(Seconds.of(5)), // TODO: tune shoot time
            pathSlices.get(2), // drive to outpost
            Commands.waitTime(Seconds.of(4)), // balls dump
            pathSlices.get(3), // drive to shot
            shootSequence()
                .withTimeout(Seconds.of(4)), // TODO: tune shoot time
            pathSlices.get(4), // drive to prepare climb
            climber.deploy(),
            pathSlices.get(5), // approach climb
            climber.climb()
        );
    }

    public Command trenchSSDepot() {
        var pathSlices = makeSlicedPath("trenchSSDepot", 3);

        return Commands.sequence(
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
        );
    }
}
