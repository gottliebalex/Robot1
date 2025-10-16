package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class WristCommands {

  public static Command Stowed(WristSubsystem wrist) {
    var target = SubsystemConstants.WristPosition.Stowed.angle();
    return Commands.deadline(wrist.setAngle(target), wrist.waitUntilAtAngle(target));
  }

  // public static Command AlgaeIntake(WristSubsystem wrist) {
  //   var target = SubsystemConstants.WristPosition.AlgaeGroundIntake.angle();
  //   return Commands.sequence(wrist.setAngle(target), wrist.waitUntilAtAngle(target));
  // }

  // public static Command TestWrist(WristSubsystem wrist) {
  //   var target = SubsystemConstants.WristPosition.Test.angle();
  //   return Commands.sequence(wrist.setAngle(target), wrist.waitUntilAtAngle(target));
  // }

  private final WristSubsystem wrist;

  public WristCommands(WristSubsystem wrist) {
    this.wrist = wrist;
  }

  public Command algaeIdle() {
    return Commands.run(
            () -> wrist.setAngleHW(SubsystemConstants.WristPosition.AlgaeTransit.angle()), wrist)
        .withName("Wrist Idle (Algae)");
  }

  public Command idle() {
    return Commands.run(
            () -> wrist.setAngleHW(SubsystemConstants.WristPosition.Stowed.angle()), wrist)
        .withName("Wrist Idle (Stowed)");
  }
}
