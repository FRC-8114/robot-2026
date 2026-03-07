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
import frc.robot.subsystems.turret.Turret;
import frc.robot.supersystems.ShooterSupersystem;

public class Autos {
    private Intake intake;
    private ShooterSupersystem shooter;
    private Climber climber;

    private static ArrayList<Command> makeSlicedPath(String choreoTraj, Integer num_slices) {
        ArrayList<Command> slices = new ArrayList<Command>();

        for (int i = 0; i <= num_slices; i++) {
            try {
                slices.add(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(choreoTraj, i)));
            } catch (FileVersionException | IOException | org.json.simple.parser.ParseException e) {
                e.printStackTrace();
            }
        }

        return slices;
    }

    private Command shootSequence() {
        return Commands.race(
            Commands.none(), /* turret.fullShootSequence() */
            Commands.waitSeconds(5)  
        ); // TODO
    }

    private Command prepareClimb() {
        return climber.deploy();
    }
    
    public Autos(Intake intake) {
        this.intake = intake;
    }

    // public Command depotShootAuto() {
    //     ArrayList<Command> pathSlices = makeSlicedPath("depot", 2);

    //     return Commands.sequence(
    //         Commands.race(
    //             intake.deploy(), // runs forever, then stops rollers once followPath finishes
    //             pathSlices.get(0)
    //         ),
    //         shootSequence(),
    //         pathSlices.get(1),
    //         climber.deploy(),
    //         pathSlices.get(2),
    //         climber.climb()
    //     );
    // }

    // public Command outpostShootAuto() {
    //     ArrayList<Command> pathSlices = makeSlicedPath("outpost", 3);

    //     return Commands.sequence(
    //         pathSlices.get(0),
    //         Commands.waitSeconds(2),
    //         pathSlices.get(1),
    //         pathSlices.get(2),
    //         pathSlices.get(3),
    //         Commands.none() /* climber.climb() */
    //     );
    // }

    public Command trenchSSDepot() {
        var pathSlices = makeSlicedPath("trenchSSDepot", 3);

        return Commands.sequence(
            Commands.race( // collect balls from alliance zone
                intake.intake(),
                pathSlices.get(0)
            ),
            shootSequence()
                .withTimeout(Seconds.of(0)), // TODO: tune shoot time
            Commands.parallel( // collect balls from depot while shooting
                pathSlices.get(1), 
                shootSequence()
                    .withTimeout(Seconds.of(0)) // TODO: tune shoot time
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
