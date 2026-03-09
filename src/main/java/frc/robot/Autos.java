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
import frc.robot.subsystems.intake.Intake;
import frc.robot.supersystems.ShooterSupersystem;

public class Autos {
    private Intake intake;
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
        System.out.println("SHOOOOOTING!!!!!!!");
        return Commands.waitUntil(() -> false);
    }

    public Autos(Intake intake, Climber climber) {
        this.intake = intake;
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

    public Command trenchSSDepot() {
        var pathSlices = makeSlicedPath("trenchSSDepot", 3);

        return Commands.sequence(
            Commands.race( // collect balls from alliance zone
                intake.intake(),
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

    public Command trenchSSOutpost() {
        var pathSlices = makeSlicedPath("trenchSSOutpost", 3);

        return Commands.sequence(
            Commands.race( // collect balls from alliance zone
                intake.intake(),
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
        );
    }
}
