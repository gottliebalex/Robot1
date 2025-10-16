package frc.robot.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef.AlgaeMode;
import frc.robot.FieldConstants.Reef.PipeSide;
import frc.robot.GamePieceState;
import frc.robot.sensors.CoralSensor;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.SubsystemConstants.ClawVoltages;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.function.BooleanSupplier;

/** Composite commands for intaking coral and algae. */
public final class IntakeCommands {
  // algae detection
  private static final double DEFAULT_ALGAE_TRIP_CURRENT_AMPS = 25.0;
  private static final double DEFAULT_ALGAE_DEBOUNCE_S = 0.10;
  private static final double DEFAULT_START_IGNORE_S = 0.15;

  // private final ClawSubsystem claw;
  // private final ClawSubsystem claw;

  /**
   * Intake coral: runs intake and end effector at specified speeds, moves elevator to intake height
   * until the coral sensor is tripped, then returns elevator to the stowed (down) height. Also sets
   * robot to CORAL mode.
   */
  public static Command intakeCoral(
      CoralIntakeSubsystem intake,
      ClawSubsystem claw,
      ElevatorSubsystem elevator,
      CoralSensor sensor,
      double volts,
      double intakeSpeed) {

    var setCoralMode = ClawCommands.setCoralMode();
    var intakeHeight = SubsystemConstants.ElevatorPosition.Intake.distance();
    var stowHeight = SubsystemConstants.ElevatorPosition.Down.distance();

    // Motors to run during intake
    var runMotors =
        Commands.parallel(
            ClawCommands.runRollers(claw, SubsystemConstants.ClawVoltages.CORAL_INTAKE),
            CoralIntakeSubsystem.runIntake(intake, volts));

    // Drive elevator toward intake height; completes when sensor trips (deadline)
    var approachUntilTripped =
        Commands.deadline(
            Commands.waitUntil(sensor::isTripped), // deadline
            runMotors,
            elevator.setHeight(intakeHeight));
    // After trip, ensure sim flag is cleared so it doesn't latch in SIM
    var clearSimTrip =
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
            Commands.runOnce(intake::stop, intake), Commands.runOnce(claw::stop, claw));

    return Commands.sequence(
            setCoralMode,
            // Run motors and move elevator toward intake height until sensor trips
            approachUntilTripped,
            // After trip, stop motors, clear sim flag, then stow elevator
            stopMotors,
            clearSimTrip,
            moveDown)
        .finallyDo(
            interrupted -> {
              try {
                intake.stop();
              } catch (Exception ignored) {
              }
              try {
                claw.stop();
              } catch (Exception ignored) {
              }
              // If canceled (e.g., second press), clear mode and any sim-detected state
              if (interrupted) {
                try {
                  GamePieceState.setMode(GamePieceState.Mode.NONE);
                } catch (Exception ignored) {
                }
                try {
                  sensor.setSimTripped(false);
                } catch (Exception ignored) {
                }
              } else {
                GamePieceState.setMode(GamePieceState.Mode.CORAL);
              }
            })
        .withName("Intake Coral");
  }

  /**
   * Approach algae at the nearest reef face using either SUPERCYCLE or GRAB standoff. Sets elevator
   * and wrist to appropriate mode-specific setpoints while aligning.
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
                      elevator.waitUntilAtHeight(elevatorTarget),
                      elevator.setHeight(elevatorTarget)),
                  Commands.deadline(
                      wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));
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
            Commands.deadline(
                elevator.waitUntilAtHeight(eleVatorTarget), elevator.setHeight(eleVatorTarget)),
            Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

    return Commands.sequence(intermediateposition, reachTargets);
  }

  /**
   * Intake algae at a given duty cycle until a current spike is detected for a sustained period. On
   * detection, stops the motor and sets the global game piece mode to ALGAE.
   */
  // public static Command intakeAlgae(
  //   ClawSubsystem claw,
  //   double volts,
  //   double rampS,
  //   double timeoutS) {
  //   return intakeAlgae(claw, volts, rampS, timeoutS, DEFAULT_ALGAE_TRIP_CURRENT_AMPS,
  //   DEFAULT_ALGAE_DEBOUNCE_S, ClawSubsystem.algaeTripSubscriber);
  // }
  /**
   * Intake algae with current trip and debounce
   *
   * @param volts
   * @param tripCurrentAmps Current threshold in amps considered as "algae engaged"
   * @param debounceSeconds Time that current must remain above threshold to count as engaged
   */
  public static Command intakeAlgae(
      ClawSubsystem claw,
      ClawVoltages voltages,
      double tripCurrentAmps,
      double debounceSeconds,
      BooleanSupplier forceTrip) {

    Debouncer debouncer = new Debouncer(debounceSeconds, Debouncer.DebounceType.kRising);
    Timer ignoreTimer = new Timer();
    ignoreTimer.restart();

    return ClawCommands.runRollers(claw, voltages)
        .until(
            () -> {
              // Refresh the stator current once per loop, then read
              var stator = claw.clawMotor.getStatorCurrent();
              BaseStatusSignal.refreshAll(stator);
              double amps = stator.getValueAsDouble();

              boolean aboveThreshhold =
                  ignoreTimer.hasElapsed(DEFAULT_START_IGNORE_S) && amps >= tripCurrentAmps;
              return debouncer.calculate(aboveThreshhold) || forceTrip.getAsBoolean();
            })
        .andThen(Commands.runOnce(() -> GamePieceState.setMode(GamePieceState.Mode.ALGAE)))
        .withName(
            String.format(
                "EndEffector intakeAlgae (V=%.1f, trip=%.1fA, debounce=%.2fs)",
                voltages.volts().in(edu.wpi.first.units.Units.Volts),
                tripCurrentAmps,
                debounceSeconds));
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
