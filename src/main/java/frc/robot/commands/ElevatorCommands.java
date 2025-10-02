package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

// use the scorecommands command factory for any scoring commands these are mainly for
// troubleshooting or quick testing

public class ElevatorCommands {
  private ElevatorCommands() {}

  public static Command Zero(ElevatorSubsystem elevator) {
    return elevator.setHeight(SubsystemConstants.ElevatorPosition.Zero.distance());
  }

  public static Command Down(ElevatorSubsystem elevator) {
    return elevator.setHeight(SubsystemConstants.ElevatorPosition.Down.distance());
  }

  public static Command L2Score(ElevatorSubsystem elevator) {
    return elevator.setHeight(SubsystemConstants.ElevatorPosition.L2.distance());
  }

  public static Command L4Score(ElevatorSubsystem elevator) {
    return elevator.setHeight(SubsystemConstants.ElevatorPosition.L4.distance());
  }
}
