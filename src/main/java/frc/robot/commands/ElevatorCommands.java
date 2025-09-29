package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

// These are used for easy tuning

public class ElevatorCommands {
  private ElevatorCommands() {}

  public static Command Zero(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.Zero.distance());
  }

  public static Command Down(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.Down.distance());
  }

  public static Command L2Score(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L2.distance());
  }

  public static Command L4Score(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L4.distance());
  }
}
