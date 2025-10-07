package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.Reef.AlgaePlacement;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/** Composite commands for removing/intaking algae from the reef. */
public final class AlgaeCommands {
  private AlgaeCommands() {}

  /**
   * Aligns to the algae on the nearest reef face and runs the end effector to intake it. Uses the
   * configured per-face algae placement (level + side) from FieldConstants.Reef.
   */
  public static Command intakeReefAlgae(
      Drive drive,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      EndEffectorSubsystem endEffector) {

    // Defer computation of the specific branch/placement until scheduling
    return Commands.defer(
            () -> {
              Reef.Branch branch = FieldConstants.Reef.nearestBranch(drive.getPose());
              AlgaePlacement placement = FieldConstants.Reef.getAlgaePlacement(branch);

              // Determine mechanism targets from placement
              var elevatorTarget =
                  switch (placement.level()) {
                    case L1 -> SubsystemConstants.ElevatorPosition.L1.distance();
                    case L2 -> SubsystemConstants.ElevatorPosition.L2Algae.distance();
                    case L3 -> SubsystemConstants.ElevatorPosition.L3Algae.distance();
                    case L4 -> SubsystemConstants.ElevatorPosition.L4.distance();
                  };
              var wristTarget =
                  switch (placement.level()) {
                    case L1 -> SubsystemConstants.WristPosition.L1Score.angle();
                    case L2 -> SubsystemConstants.WristPosition.L2Score.angle();
                    case L3 -> SubsystemConstants.WristPosition.L3Score.angle();
                    case L4 -> SubsystemConstants.WristPosition.L4Score.angle();
                  };

              var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();
              var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();

              Command align = DriveCommands.alignToNearestAllianceReefAlgae(drive);

              // Run alignment and approach mechanism setpoints together
              Command reachTargets =
                  Commands.parallel(
                      align,
                      Commands.deadline(
                          elevator.waitUntilAtHeight(elevatorTarget),
                          elevator.setHeight(elevatorTarget)),
                      Commands.deadline(
                          wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

              // Spin end effector to take algae off the post briefly
              Command runEffector =
                  endEffector.runDuty(SubsystemConstants.DEFAULT_END_EFFECTOR_SPEED);

              // Intake window then stow
              Command performIntake =
                  Commands.deadline(Commands.waitSeconds(0.75), runEffector)
                      .andThen(Commands.runOnce(endEffector::stop, endEffector));

              Command stow =
                  Commands.deadline(
                      Commands.waitUntil(
                          () -> elevator.atHeight(stowElevator) && wrist.atAngle(stowWrist)),
                      elevator.setHeight(stowElevator),
                      wrist.setAngle(stowWrist));

              return Commands.sequence(reachTargets, performIntake, stow)
                  .withName("Intake Reef Algae");
            },
            java.util.Set.of(drive, elevator, wrist, endEffector))
        .withName("Intake Reef Algae (deferred)");
  }
}
