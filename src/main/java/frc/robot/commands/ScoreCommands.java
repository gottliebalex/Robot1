package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/** Composite scoring commands for reef levels. */
public final class ScoreCommands {
  private ScoreCommands() {}

  /**
   * Score at a reef level: auto-align to the level standoff while moving mechanisms. Once aligned
   * and at setpoints, wait 1s to simulate scoring, then stow wrist and lower elevator.
   *
   * @param level Reef level to score at (2, 3, or 4)
   */
  public static Command scoreReefLevel(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist, int level) {
    // Map level -> elevator height + wrist angle
    var elevatorCmd =
        switch (level) {
          case 2 -> elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L2.distance());
          case 3 -> elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L3.distance());
          case 4 -> elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L4.distance());
          default -> elevator.setHeight(ElevatorSubsystem.ElevatorPosition.Down.distance());
        };

    var wristCmd =
        switch (level) {
          case 2 -> wrist.setAngle(WristSubsystem.WristPosition.L2Score.angle());
          case 3 -> wrist.setAngle(WristSubsystem.WristPosition.L3Score.angle());
          case 4 -> wrist.setAngle(WristSubsystem.WristPosition.L4Score.angle());
          default -> wrist.setAngle(WristSubsystem.WristPosition.Stowed.angle());
        };

    Command alignCmd = DriveCommands.alignToNearestAllianceReefFace(drive, level);

    // Run alignment + mechanisms together, finish when all reach target
    Command reachTargets = Commands.parallel(alignCmd, elevatorCmd, wristCmd);

    // Simulate scoring, then stow + lower
    Command postScore =
        Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.parallel(
                frc.robot.commands.WristCommands.Stowed(wrist),
                frc.robot.commands.ElevatorCommands.Down(elevator)));

    return Commands.sequence(reachTargets, postScore).withName("Score L" + level);
  }

  public static Command scoreL2(Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 2).withName("Score L2");
  }

  public static Command scoreL3(Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 3).withName("Score L3");
  }

  public static Command scoreL4(Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 4).withName("Score L4");
  }
}
