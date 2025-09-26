package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/** Autonomous helpers and routines. */
public final class Autos {
  private Autos() {}

  public static Command choreoStartJ(Drive drive) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Start-J");

      Command reset =
          path.getStartingHolonomicPose()
              .<Command>map(AutoBuilder::resetOdom)
              .orElse(Commands.none());

      return Commands.sequence(reset, AutoBuilder.followPath(path)).withName("Start-J (Choreo)");
    } catch (IOException | ParseException | FileVersionException e) {
      return Commands.print("Failed to load Choreo path Start-J: " + e.getMessage());
    }
  }

  /** Creates a command that runs Start-J, waits 1.5s, then runs J-Station. */
  public static Command choreoStartJThenJStation(Drive drive) {
    try {
      PathPlannerPath startJ = PathPlannerPath.fromChoreoTrajectory("Start-J");
      PathPlannerPath jStation = PathPlannerPath.fromChoreoTrajectory("J-Station");

      Command reset =
          startJ
              .getStartingHolonomicPose()
              .<Command>map(AutoBuilder::resetOdom)
              .orElse(Commands.none());

      return Commands.sequence(
              reset,
              AutoBuilder.followPath(startJ),
              Commands.waitSeconds(1.5),
              AutoBuilder.followPath(jStation))
          .withName("Start-J -> J-Station (Choreo)");
    } catch (IOException | ParseException | FileVersionException e) {
      return Commands.print("Failed to load Choreo paths Start-J/J-Station: " + e.getMessage());
    }
  }
}
