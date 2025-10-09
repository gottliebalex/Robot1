package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.FieldConstants.Reef.AlgaeMode;
import frc.robot.FieldConstants.Reef.Branch;
import frc.robot.FieldConstants.Reef.PipeSide;
import frc.robot.FieldConstants;
import frc.robot.GamePiece;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/** Composite scoring commands for reef levels. */
public final class ScoreCommands {
  private ScoreCommands() {}

  /**
   * Score at a reef level: auto-align to the level standoff while moving mechanisms. Once aligned
   * and at setpoints, wait 0.5s to simulate scoring, then stow wrist and lower elevator.
   *
   * @param level Reef level to score at (2, 3, or 4)
   */
  public static Command scoreReefLevel(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist, int level) {
    // Map level -> elevator height + wrist angle
    var elevatorTarget =
        switch (level) {
          case 2 -> SubsystemConstants.ElevatorPosition.L2.distance();
          case 3 -> SubsystemConstants.ElevatorPosition.L3.distance();
          case 4 -> SubsystemConstants.ElevatorPosition.L4.distance();
          default -> SubsystemConstants.ElevatorPosition.Down.distance();
        };

    var wristTarget =
        switch (level) {
          case 2 -> SubsystemConstants.WristPosition.L2Score.angle();
          case 3 -> SubsystemConstants.WristPosition.L3Score.angle();
          case 4 -> SubsystemConstants.WristPosition.L4Score.angle();
          default -> SubsystemConstants.WristPosition.Stowed.angle();
        };

    var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();
    var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();

    Command alignCmd = DriveCommands.alignToNearestAllianceReefFace(drive, level);

    // Run alignment + mechanisms together, finish when all reach target
    Command reachTargets =
        Commands.parallel(
            alignCmd,
            // Keep commanding the setpoint until the wait completes
            Commands.deadline(
                elevator.waitUntilAtHeight(elevatorTarget), elevator.setHeight(elevatorTarget)),
            Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

    // Simulate scoring, then stow + lower
    Command postScore =
        Commands.sequence(
            Commands.waitSeconds(0.50),
            Commands.parallel(
                Commands.deadline(elevator.waitUntilAtHeight(stowElevator)).withTimeout(1),
                elevator.setHeight(stowElevator),
                Commands.deadline(
                    wrist.waitUntilAtAngle(stowWrist).withTimeout(1), wrist.setAngle(stowWrist))));

    return Commands.sequence(
            reachTargets, postScore, Commands.runOnce(() -> GamePiece.setMode(GamePiece.Mode.NONE)))
        .withName("Score L" + level);
  }

  /** Score at a reef level targeting a specific pipe side (left/right). */
  public static Command scoreReefLevel(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist, int level, PipeSide side) {
    // Map level -> elevator height + wrist angle
    var elevatorTarget =
        switch (level) {
          case 2 -> SubsystemConstants.ElevatorPosition.L2.distance();
          case 3 -> SubsystemConstants.ElevatorPosition.L3.distance();
          case 4 -> SubsystemConstants.ElevatorPosition.L4.distance();
          default -> SubsystemConstants.ElevatorPosition.Down.distance();
        };

    var wristTarget =
        switch (level) {
          case 2 -> SubsystemConstants.WristPosition.L2Score.angle();
          case 3 -> SubsystemConstants.WristPosition.L3Score.angle();
          case 4 -> SubsystemConstants.WristPosition.L4Score.angle();
          default -> SubsystemConstants.WristPosition.Stowed.angle();
        };

    var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();
    var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();

    Command alignCmd = DriveCommands.alignToNearestAllianceReefFace(drive, level, side);

    // Run alignment + mechanisms together
    Command reachTargets =
        Commands.parallel(
            alignCmd,
            Commands.deadline(
                elevator.waitUntilAtHeight(elevatorTarget), elevator.setHeight(elevatorTarget)),
            Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

    // Simulate scoring until the end effector is programed, then stow + lower
    Command postScore =
        Commands.sequence(
            Commands.waitSeconds(0.50),
            Commands.deadline(
                Commands.waitUntil(
                    () -> elevator.atHeight(stowElevator) && wrist.atAngle(stowWrist)),
                elevator.setHeight(stowElevator),
                wrist.setAngle(stowWrist))
            // .withTimeout(1)
            );

    return Commands.sequence(
            reachTargets, postScore, Commands.runOnce(() -> GamePiece.setMode(GamePiece.Mode.NONE)))
        .withName("Score L" + level + " (" + side + ")");
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

  // Convenience helpers for left/right pipe selection
  public static Command scoreL2Left(Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 2, PipeSide.LEFT).withName("Score L2 Left");
  }

  public static Command scoreL2Right(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 2, PipeSide.RIGHT).withName("Score L2 Right");
  }

  public static Command scoreL3Left(Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 3, PipeSide.LEFT).withName("Score L3 Left");
  }

  public static Command scoreL3Right(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 3, PipeSide.RIGHT).withName("Score L3 Right");
  }

  public static Command scoreL4Left(Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 4, PipeSide.LEFT).withName("Score L4 Left");
  }

  public static Command scoreL4Right(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist) {
    return scoreReefLevel(drive, elevator, wrist, 4, PipeSide.RIGHT).withName("Score L4 Right");
  }

  /**
   * Score at a reef level (no side) and, if supercycle is enabled, immediately approach algae,
   * align to SUPERCYCLE standoff, intake algae, then stow elevator/wrist.
   */
  // public static Command scoreReefLevel(
  //     Drive drive,
  //     EndEffectorSubsystem endEffector,
  //     ElevatorSubsystem elevator,
  //     WristSubsystem wrist,
  //     int level) {
  //   // If supercycle is OFF or no end effector present, use regular score
  //   if (!GamePiece.isSupercycleEnabled() || endEffector == null) {
  //     return scoreReefLevel(drive, elevator, wrist, level);
  //   }

  //   // Build the coral score approach without stowing after scoring
  //   var elevatorTarget =
  //       switch (level) {
  //         case 2 -> SubsystemConstants.ElevatorPosition.L2.distance();
  //         case 3 -> SubsystemConstants.ElevatorPosition.L3.distance();
  //         case 4 -> SubsystemConstants.ElevatorPosition.L4.distance();
  //         default -> SubsystemConstants.ElevatorPosition.Down.distance();
  //       };
  //   var wristTarget =
  //       switch (level) {
  //         case 2 -> SubsystemConstants.WristPosition.L2Score.angle();
  //         case 3 -> SubsystemConstants.WristPosition.L3Score.angle();
  //         case 4 -> SubsystemConstants.WristPosition.L4Score.angle();
  //         default -> SubsystemConstants.WristPosition.Stowed.angle();
  //       };

  //   Command alignCoral = DriveCommands.alignToNearestAllianceReefFace(drive, level);
  //   Command reachCoralTargets =
  //       Commands.parallel(
  //           alignCoral,
  //           Commands.deadline(
  //               elevator.waitUntilAtHeight(elevatorTarget), elevator.setHeight(elevatorTarget)),
  //           Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

  //   // Follow-up: approach algae (SC), intake, then stow
  //   var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();
  //   var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();
  //   Command approachSC =
  //       IntakeCommands.approachReefAlgae(drive, elevator, wrist, AlgaeMode.SUPERCYCLE);
  //   Command intakeAlgae =
  //       endEffector.intakeAlgae(SubsystemConstants.DEFAULT_END_EFFECTOR_SPEED);
  //   Command stowAfterIntake =
  //       Commands.deadline(
  //           Commands.waitUntil(() -> elevator.atHeight(stowElevator) && wrist.atAngle(stowWrist)),
  //           endEffector.holdAlgae(),
  //           elevator.setHeight(stowElevator),
  //           wrist.setAngle(stowWrist));

  //   return Commands.sequence(
  //           reachCoralTargets, Commands.waitSeconds(0.50), approachSC, intakeAlgae, stowAfterIntake)
  //       .withName("Score L" + level + " + Supercycle");
  // }

  /**
   * Score at a reef level on a side and, if supercycle is enabled, immediately approach algae,
   * align to SUPERCYCLE standoff, intake algae, then stow elevator/wrist.
   */
  // public static Command scoreReefLevel(
  //     Drive drive,
  //     EndEffectorSubsystem endEffector,
  //     ElevatorSubsystem elevator,
  //     WristSubsystem wrist,
  //     int level,
  //     PipeSide side) {
  //   // If supercycle is OFF or no end effector present, use regular score
  //   if (!GamePiece.isSupercycleEnabled() || endEffector == null) {
  //     return scoreReefLevel(drive, elevator, wrist, level, side);
  //   }

  //   // Build the coral score approach without stowing after scoring
  //   var elevatorTarget =
  //       switch (level) {
  //         case 2 -> SubsystemConstants.ElevatorPosition.L2.distance();
  //         case 3 -> SubsystemConstants.ElevatorPosition.L3.distance();
  //         case 4 -> SubsystemConstants.ElevatorPosition.L4.distance();
  //         default -> SubsystemConstants.ElevatorPosition.Down.distance();
  //       };
  //   var wristTarget =
  //       switch (level) {
  //         case 2 -> SubsystemConstants.WristPosition.L2Score.angle();
  //         case 3 -> SubsystemConstants.WristPosition.L3Score.angle();
  //         case 4 -> SubsystemConstants.WristPosition.L4Score.angle();
  //         default -> SubsystemConstants.WristPosition.Stowed.angle();
  //       };
  //   Command alignCoral = DriveCommands.alignToNearestAllianceReefFace(drive, level, side);
  //   Command reachCoralTargets =
  //       Commands.parallel(
  //           alignCoral,
  //           Commands.deadline(
  //               elevator.waitUntilAtHeight(elevatorTarget), elevator.setHeight(elevatorTarget)),
  //           Commands.deadline(wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));

  //   // Follow-up: approach algae (SC), intake, then stow
  //   var stowElevator = SubsystemConstants.ElevatorPosition.Down.distance();
  //   var stowWrist = SubsystemConstants.WristPosition.Stowed.angle();
  //   Command approachSC =
  //       IntakeCommands.approachReefAlgae(drive, elevator, wrist, AlgaeMode.SUPERCYCLE);
  //   Command intakeAlgae =
  //       endEffector.intakeAlgae(SubsystemConstants.DEFAULT_END_EFFECTOR_SPEED);
  //   Command stowAfterIntake =
  //       Commands.deadline(
  //           Commands.waitUntil(() -> elevator.atHeight(stowElevator) && wrist.atAngle(stowWrist)),
  //           endEffector.holdAlgae(),
  //           elevator.setHeight(stowElevator),
  //           wrist.setAngle(stowWrist));

  //   return Commands.sequence(
  //           reachCoralTargets, Commands.waitSeconds(0.50), approachSC, intakeAlgae, stowAfterIntake)
  //       .withName("Score L" + level + " (" + side + ") + Supercycle");
  // }

  public static Command scoreReefLevel(
    Drive drive, 
    ElevatorSubsystem elevator, 
    WristSubsystem wrist, 
    EndEffectorSubsystem endeffector,
    int level, 
    AlgaeMode mode,
    PipeSide side,
    double volts,
    double tripCurrentAmps,
    double debounceSeconds) {
      //coral approaches
      var elevatorTarget =
        switch (level) {
          case 2 -> SubsystemConstants.ElevatorPosition.L2.distance();
          case 3 -> SubsystemConstants.ElevatorPosition.L4.distance();
          case 4 -> SubsystemConstants.ElevatorPosition.L4.distance();
          default -> SubsystemConstants.ElevatorPosition.Down.distance();
        };

        var wristTarget =
        switch (level) {
          case 2 -> SubsystemConstants.WristPosition.L2Score.angle();
          case 3 -> SubsystemConstants.WristPosition.L3Score.angle();
          case 4 -> SubsystemConstants.WristPosition.L4Score.angle();
          default -> SubsystemConstants.WristPosition.Stowed.angle();
        };

    Command coralAlign = DriveCommands.alignToNearestAllianceReefFace(drive, level, side);

    Command algaeAlign = DriveCommands.alignToNearestAlgaePose(drive, AlgaeMode.SUPERCYCLE);

    Command reachCoralSetpoints =
        Commands.parallel(
          coralAlign,
          Commands.deadline(
            elevator.waitUntilAtHeight(elevatorTarget), elevator.setHeight(elevatorTarget)),
          Commands.deadline(
          wrist.waitUntilAtAngle(wristTarget), wrist.setAngle(wristTarget)));
    
    Command scoreCoral = 
        endeffector.scoreCoral(level).withTimeout(0.4);
        
    var nearest = FieldConstants.Reef.nearestBranch(drive.getPose());

    int baseLevel = FieldConstants.Reef.algaeBaseLevel(nearest, AlgaeMode.SUPERCYCLE);

    var SCElevatorTarget = (baseLevel == 2)
        ? SubsystemConstants.ElevatorPosition.L2SCAlgae.distance()
        : SubsystemConstants.ElevatorPosition.L3SCAlgae.distance();

    var SCwristTarget = SubsystemConstants.WristPosition.ReefSCAlgae.angle();

    Command intakeAlgae =
      Commands.sequence(endeffector.intakeAlgae(volts, tripCurrentAmps, debounceSeconds));

    Command SCorNOT;
      
    if (mode == AlgaeMode.SUPERCYCLE){
      SCorNOT=
        Commands.parallel(
              algaeAlign,
              Commands.deadline(
                elevator.waitUntilAtHeight(SCElevatorTarget), 
                elevator.setHeight(SCElevatorTarget)),
              Commands.deadline(
                wrist.waitUntilAtAngle(SCwristTarget), 
                wrist.waitUntilAtAngle(SCwristTarget)),
              intakeAlgae
              );
    }
      Command supercycle =
          

    return Commands.sequence(
      reachCoralSetpoints, scoreCoral, 
    )
        
    }
}


