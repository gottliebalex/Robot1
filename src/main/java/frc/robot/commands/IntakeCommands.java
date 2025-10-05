package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePiece;
import frc.robot.sensors.CoralSensor;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;

/** Composite commands for intaking coral. */
public final class IntakeCommands {
  private IntakeCommands() {}

  /**
   * Intake coral: runs intake and end effector at specified speeds, moves elevator to intake height
   * until the coral sensor is tripped, then returns elevator to the stowed (down) height. Also sets
   * robot to CORAL mode.
   */
  public static Command intakeCoral(
      CoralIntakeSubsystem intake,
      EndEffectorSubsystem endEffector,
      ElevatorSubsystem elevator,
      CoralSensor sensor,
      double intakeSpeed,
      double endEffectorSpeed) {
    var intakeHeight = SubsystemConstants.ElevatorPosition.Intake.distance();
    var stowHeight = SubsystemConstants.ElevatorPosition.Down.distance();

    var setMode = Commands.runOnce(() -> GamePiece.setMode(GamePiece.Mode.CORAL));

    var runIntake = intake.runDuty(intakeSpeed);
    var runEndEffector = endEffector.runDuty(endEffectorSpeed);

    // Move to intake height while motors run until sensor trips
    var moveToIntakeUntilTripped =
        Commands.deadline(Commands.waitUntil(sensor::isTripped), elevator.setHeight(intakeHeight));
    // After trip, ensure sim flag is cleared so it doesn't latch in SIM
    var clearSimAfterTrip =
        Commands.runOnce(
            () -> {
              try {
                sensor.setSimTripped(false);
              } catch (Exception ignored) {
              }
            });

    // Then move elevator back down to stowed
    var moveDown =
        Commands.deadline(
            elevator.waitUntilAtHeight(SubsystemConstants.ElevatorPosition.Down.distance()),
            elevator.setHeight(SubsystemConstants.ElevatorPosition.Down.distance()));

    var stopMotors =
        Commands.parallel(
            Commands.runOnce(intake::stop, intake),
            Commands.runOnce(endEffector::stop, endEffector));

    return Commands.sequence(
            setMode,
            // Use a deadline group so motors run while approaching intake height,
            // but the group exits as soon as the sensor trips. Then clear sim flag.
            Commands.deadline(moveToIntakeUntilTripped, runIntake, runEndEffector)
                .andThen(clearSimAfterTrip),
            moveDown,
            stopMotors)
        .finallyDo(
            interrupted -> {
              try {
                intake.stop();
              } catch (Exception ignored) {
              }
              try {
                endEffector.stop();
              } catch (Exception ignored) {
              }
              // If canceled (e.g., second press), clear mode and any sim-detected state
              if (interrupted) {
                try {
                  GamePiece.setMode(GamePiece.Mode.NONE);
                } catch (Exception ignored) {
                }
                try {
                  sensor.setSimTripped(false);
                } catch (Exception ignored) {
                }
              }
            })
        .withName("Intake Coral");
  }

  /** Utility to simulate a brief coral detection pulse via SmartDashboard. */
  public static Command simulateCoralDetectionPulse(CoralSensor sensor) {
    return Commands.sequence(
            Commands.runOnce(() -> sensor.setSimTripped(true)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> sensor.setSimTripped(false)))
        .withName("Simulate Coral Detection");
  }
}
