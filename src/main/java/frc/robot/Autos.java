package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.SubsystemConstants.ElevatorPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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

  public static Command choreoStartJThenJStation(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    try {
      PathPlannerPath startJoffset = PathPlannerPath.fromChoreoTrajectory("Start-Joffset");
      PathPlannerPath jStation = PathPlannerPath.fromChoreoTrajectory("J-Station");

      Command reset =
          startJoffset
              .getStartingHolonomicPose()
              .<Command>map(AutoBuilder::resetOdom)
              .orElse(Commands.none());

      return Commands.sequence(
              reset,
              Commands.parallel(AutoBuilder.followPath(startJoffset),
              elevator.waitUntilAtHeight(SubsystemConstants.ElevatorPosition.L3)),              
              ScoreCommands.scoreL4Right(drive, elevator, wrist),
              AutoBuilder.followPath(jStation))
          .withName("Start-J -> J-Station (Choreo)");
    } catch (IOException | ParseException | FileVersionException e) {
      return Commands.print("Failed to load Choreo paths Start-J/J-Station: " + e.getMessage());
    }
  }
}
