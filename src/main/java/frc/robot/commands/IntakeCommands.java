package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef.PipeSide;
import frc.robot.FieldConstants.Reef.AlgaeMode;
import frc.robot.GamePiece;
import frc.robot.sensors.CoralSensor;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/** Composite commands for intaking coral and algae. */
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

    // move elevator back down to stowed
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

  /**
   * Approach algae at the nearest reef face using either SUPERCYCLE or GRAB standoff.
   * Sets elevator and wrist to appropriate mode-specific setpoints while aligning.
   */
  public static Command approachReefAlgae(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist, AlgaeMode mode) {
    return new DeferredCommand(
        () -> {
          var nearest = FieldConstants.Reef.nearestBranch(drive.getPose());
          int baseLevel = FieldConstants.Reef.algaeBaseLevel(nearest, mode); // 2 or 3

          var elevatorTarget =
              (mode == AlgaeMode.GRAB)
                  ? (baseLevel == 2
                      ? SubsystemConstants.ElevatorPosition.L2GrabAlgae.distance()
                      : SubsystemConstants.ElevatorPosition.L3GrabAlgae.distance())
                  : (baseLevel == 2
                      ? SubsystemConstants.ElevatorPosition.L2.distance()
                      : SubsystemConstants.ElevatorPosition.L3.distance());

          var wristTarget =
              (mode == AlgaeMode.GRAB)
                  ? SubsystemConstants.WristPosition.ReefGrabAlgae.angle()
                  : SubsystemConstants.WristPosition.ReefSCAlgae.angle();

          Command align = DriveCommands.alignToNearestAlgaePose(drive, mode);

          return Commands.parallel(
              align,
              Commands.deadline(
                  elevator.waitUntilAtHeight(elevatorTarget), elevator.setHeight(elevatorTarget)),
              Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));
        },
        java.util.Set.of(drive, elevator, wrist))
        .withName("Approach Reef Algae (" + mode + ")");
  }

  /** Utility to simulate a brief coral detection pulse via SmartDashboard. */
  public static Command simulateCoralDetectionPulse(CoralSensor sensor) {
    return Commands.sequence(
            Commands.runOnce(() -> sensor.setSimTripped(true)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> sensor.setSimTripped(false)))
        .withName("Simulate Coral Detection");
  }

  public Command intakeReefAlgae(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist, int level, PipeSide side) {
    var eleVatorTarget =
        switch (level) {
          case 1 -> SubsystemConstants.ElevatorPosition.L2GrabAlgae.distance();
          case 2 -> SubsystemConstants.ElevatorPosition.L3GrabAlgae.distance();
          default -> SubsystemConstants.ElevatorPosition.Down.distance();
        };

    var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();

    var wristTarget = SubsystemConstants.WristPosition.ReefGrabAlgae.angle();

    var wristIntermediate = SubsystemConstants.WristPosition.GrabAlgaeIntermediate.angle();

    var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();

    Command alignCmd = DriveCommands.alignToNearestAllianceReefFace(drive, level, side);

    // Align + set the wrist to + 90 degrees (can't do it all at once or collissions happen if it
    // starts too close to the reef)
    Command intermediateposition =
        Commands.parallel(
            alignCmd,
            Commands.deadline(
                wrist.waitUntilAtAngle(wristIntermediate), wrist.setAngle(wristIntermediate)));
    Command reachTargets = 
        Commands.parallel(
          Commands.deadline(elevator.waitUntilAtHeight(eleVatorTarget), elevator.setHeight(eleVatorTarget)),
          Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

    return Commands.sequence(
      intermediateposition, reachTargets);
  }


  /**
   * Intake algae from the reef at L2/L3: align to approach standoff, move mechanisms to targets,
   * approach closer to the GRAB standoff, then run the end effector until algae is detected by
   * current. Hold while stowing mechanisms.
   *
   * <p>Level is clamped to [2,3] semantics for algae.
   */
  //   public static Command intakeReefAlgae(
  //       Drive drive,
  //       EndEffectorSubsystem endEffector,
  //       ElevatorSubsystem elevator,
  //       WristSubsystem wrist,
  //       int level,
  //       double intakePercent) {
  //     // Clamp to L2/L3 semantics
  //     int lvl = level <= 2 ? 2 : 3;

  //     var elevatorTarget =
  //         (lvl == 2)
  //             ? SubsystemConstants.ElevatorPosition.L2Algae.distance()
  //             : SubsystemConstants.ElevatorPosition.L3Algae.distance();
  //     var wristTarget =
  //         (lvl == 2)
  //             ? SubsystemConstants.WristPosition.L2Score.angle()
  //             : SubsystemConstants.WristPosition.L3Score.angle();

  //     var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();
  //     var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();

  //     // Align to approach (SCORE) standoff at the nearest reef face while mechanisms move
  //     Command alignApproach =
  //         Commands.defer(
  //             () -> {
  //               var nearest = FieldConstants.Reef.nearestBranch(drive.getPose());
  //               var target = FieldConstants.Reef.algaePose(nearest, lvl, AlgaeStandoff.GRAB);
  //               return drive.align(Drive.facingBackTo(target));
  //             },
  //             java.util.Set.of(drive));

  //     Command reachApproach =
  //         Commands.parallel(
  //             alignApproach,
  //             Commands.deadline(
  //                 elevator.waitUntilAtHeight(elevatorTarget),
  // elevator.setHeight(elevatorTarget)),
  //             Commands.deadline(wrist.waitUntilAtAngle(wristTarget),
  // wrist.setAngle(wristTarget)));

  //     // Move into GRAB standoff for intake
  //     Command alignGrab =
  //         Commands.defer(
  //             () -> {
  //               var nearest = FieldConstants.Reef.nearestBranch(drive.getPose());
  //               var target = FieldConstants.Reef.algaePose(nearest, lvl, AlgaeStandoff.GRAB);
  //               return drive.align(Drive.facingBackTo(target));
  //             },
  //             java.util.Set.of(drive));

  //     // Run end effector until current spike indicates algae captured
  //     Command intakeAlgae = endEffector.intakeAlgae(intakePercent);

  //     // Hold algae torque while stowing mechanisms
  //     Command holdAndStow =
  //         Commands.deadline(
  //             Commands.waitUntil(() -> elevator.atHeight(stowElevator) &&
  // wrist.atAngle(stowWrist)),
  //             endEffector.holdAlgae(),
  //             elevator.setHeight(stowElevator),
  //             wrist.setAngle(stowWrist));

  //     return Commands.sequence(reachApproach, alignGrab, intakeAlgae, holdAndStow)
  //         .finallyDo(
  //             interrupted -> {
  //               try {
  //                 endEffector.stop();
  //               } catch (Exception ignored) {
  //               }
  //               if (interrupted) {
  //                 try {
  //                   GamePiece.setMode(GamePiece.Mode.NONE);
  //                 } catch (Exception ignored) {
  //                 }
  //               }
  //             })
  //         .withName("Intake Reef Algae L" + lvl);
  //   }

  //   /** Overload with default intake percent. */
  //   public static Command intakeReefAlgae(
  //       Drive drive,
  //       EndEffectorSubsystem endEffector,
  //       ElevatorSubsystem elevator,
  //       WristSubsystem wrist,
  //       int level) {
  //     return intakeReefAlgae(
  //         drive, endEffector, elevator, wrist, level,
  // SubsystemConstants.DEFAULT_END_EFFECTOR_SPEED);
  //   }
}
