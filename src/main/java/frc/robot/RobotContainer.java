// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.Reef.PipeSide;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.WristCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.CoralSensor;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final CoralIntakeSubsystem coralIntake;
  private final EndEffectorSubsystem endEffector;
  private final CoralSensor coralSensor;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final GenericHID apacController = new GenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Map<Command, Pose2d> autoStartingPosesBlue = new HashMap<>();
  private final Map<Command, Pose2d> autoStartingPosesRed = new HashMap<>();
  private final LoggedDashboardChooser<StartPose> startPoseChooser;
  private final Map<StartPose, Pose2d> manualStartingPosesBlue = new HashMap<>();
  private final Map<StartPose, Pose2d> manualStartingPosesRed = new HashMap<>();
  private final LoggedNetworkNumber ReefLevel = // auto align level selection
      new LoggedNetworkNumber("Autopilot/ReefLevel", 4);
  // Selected pipe side for scoring (set by driver triggers/Y)
  private PipeSide selectedPipeSide = PipeSide.CENTER;

  /** Manual starting pose options. */
  public enum StartPose {
    NONE,
    LEFT,
    RIGHT
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Cache alliance at init and update FieldConstants' precomputed views
    FieldConstants.refreshAllianceCache();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("camera_0", robotToCamera0),
                new VisionIOPhotonVision("camera_1", robotToCamera1));

        elevator = new ElevatorSubsystem();
        wrist = new WristSubsystem();
        coralIntake = new CoralIntakeSubsystem();
        endEffector = new EndEffectorSubsystem();
        coralSensor = new CoralSensor(SubsystemConstants.CANANDCOLOR_ID);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim("camera_0", robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim("camera_1", robotToCamera1, drive::getPose));
        elevator = new ElevatorSubsystem();
        wrist = new WristSubsystem();
        coralIntake = new CoralIntakeSubsystem();
        endEffector = new EndEffectorSubsystem();
        coralSensor = new CoralSensor(SubsystemConstants.CANANDCOLOR_ID);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = null;
        wrist = null;
        coralIntake = null;
        endEffector = null;
        coralSensor = new CoralSensor();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add Choreo single-path auto: Start-J
    Command startJChoreo = Autos.choreoStartJ(drive);
    autoChooser.addOption("Start-J (Choreo)", startJChoreo);
    // Populate starting poses for Start-J if the trajectory is available
    try {
      PathPlannerPath p = PathPlannerPath.fromChoreoTrajectory("Start-J");
      p.getStartingHolonomicPose()
          .ifPresent(
              bluePose -> {
                autoStartingPosesBlue.put(startJChoreo, bluePose);
                autoStartingPosesRed.put(startJChoreo, FlippingUtil.flipFieldPose(bluePose));
              });
    } catch (Exception ignored) {
    }

    Command startJThenJStation = Autos.choreoStartJThenJStation(drive, elevator, wrist);
    autoChooser.addOption("Start-J -> J-Station (Choreo)", startJThenJStation);
    try {
      PathPlannerPath p = PathPlannerPath.fromChoreoTrajectory("Start-J");
      p.getStartingHolonomicPose()
          .ifPresent(
              bluePose -> {
                autoStartingPosesBlue.put(startJThenJStation, bluePose);
                autoStartingPosesRed.put(startJThenJStation, FlippingUtil.flipFieldPose(bluePose));
              });
    } catch (Exception ignored) {
    }

    // Manual starting pose chooser
    startPoseChooser = new LoggedDashboardChooser<>("Start Pose");
    startPoseChooser.addDefaultOption("None", StartPose.NONE);
    startPoseChooser.addOption("Left", StartPose.LEFT);
    startPoseChooser.addOption("Right", StartPose.RIGHT);
    manualStartingPosesBlue.put(StartPose.LEFT, FieldConstants.LEFT_STARTING_POSE_BLUE);
    manualStartingPosesRed.put(StartPose.LEFT, FieldConstants.LEFT_STARTING_POSE_RED);
    manualStartingPosesBlue.put(StartPose.RIGHT, FieldConstants.RIGHT_STARTING_POSE_BLUE);
    manualStartingPosesRed.put(StartPose.RIGHT, FieldConstants.RIGHT_STARTING_POSE_RED);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Default wrist hold at 0 degrees (stowed)
    if (wrist != null) {
      wrist.setDefaultCommand(
          Commands.sequence(
                  wrist.setAngle(SubsystemConstants.WristPosition.Stowed.angle()),
                  Commands.run(() -> {}, wrist))
              .withName("Wrist Default Command (0 deg)"));

      elevator.setDefaultCommand(
          Commands.sequence(
                  elevator.setHeight(SubsystemConstants.ElevatorPosition.Down.distance()),
                  Commands.run(() -> {}, elevator))
              .withName("Elevator Default Command"));
    }

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // select level standoff and triggers/Y for alignment

    new JoystickButton(apacController, 8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  ReefLevel.set(1);
                  System.out.println("Autopilot level set to L1 (APAC)");
                }));
    new JoystickButton(apacController, 9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  ReefLevel.set(2);
                  System.out.println("Autopilot level set to L2 (APAC)");
                }));
    new JoystickButton(apacController, 10)
        .onTrue(
            Commands.runOnce(
                () -> {
                  ReefLevel.set(3);
                  System.out.println("Autopilot level set to L3 (APAC)");
                }));
    new JoystickButton(apacController, 11)
        .onTrue(
            Commands.runOnce(
                () -> {
                  ReefLevel.set(4);
                  System.out.println("Autopilot level set to L4 (APAC)");
                }));

    // Left/Right triggers: choose LEFT/RIGHT pipe side and align while held
    controller.leftTrigger().onTrue(Commands.runOnce(() -> selectedPipeSide = PipeSide.LEFT));
    controller
        .leftTrigger()
        .whileTrue(
            DriveCommands.alignToNearestAllianceReefFace(
                drive, () -> getSelectedReefLevel(), PipeSide.LEFT));
    controller.rightTrigger().onTrue(Commands.runOnce(() -> selectedPipeSide = PipeSide.RIGHT));
    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.alignToNearestAllianceReefFace(
                drive, () -> getSelectedReefLevel(), PipeSide.RIGHT));

    // Y button: choose CENTER pipe and align while held
    controller.y().onTrue(Commands.runOnce(() -> selectedPipeSide = PipeSide.CENTER));
    controller
        .y()
        .whileTrue(
            DriveCommands.alignToNearestAllianceReefFace(
                drive, () -> getSelectedReefLevel(), PipeSide.CENTER));

    if (elevator != null && wrist != null) {
      // Left/Right bumpers: run scoring on LEFT/RIGHT pipe at selected level
      controller
          .leftBumper()
          .onTrue(
              Commands.defer(
                  () -> {
                    int lvl = getSelectedReefLevel();
                    if (lvl < 2) lvl = 2;
                    if (lvl > 4) lvl = 4;
                    return ScoreCommands.scoreReefLevel(drive, elevator, wrist, lvl, PipeSide.LEFT);
                  },
                  java.util.Set.of(drive, elevator, wrist)));
      controller
          .rightBumper()
          .onTrue(
              Commands.defer(
                  () -> {
                    int lvl = getSelectedReefLevel();
                    if (lvl < 2) lvl = 2;
                    if (lvl > 4) lvl = 4;
                    return ScoreCommands.scoreReefLevel(
                        drive, elevator, wrist, lvl, PipeSide.RIGHT);
                  },
                  java.util.Set.of(drive, elevator, wrist)));
      // Scoring: select level with APAC (8-11), select side with triggers (LT/RT or Y),
      // then press any level button (2/3/4) to run the score sequence for the current selection.
      // Command dynamicScore =
      //     Commands.defer(
      //         () -> {
      //           int lvl = getSelectedReefLevel();
      //           // ScoreCommands supports L2-L4; clamp here to avoid L1 mis-selection
      //           if (lvl < 2) lvl = 2;
      //           if (lvl > 4) lvl = 4;
      //           return ScoreCommands.scoreReefLevel(drive, elevator, wrist, lvl,
      // selectedPipeSide);
      //         },
      //         java.util.Set.of(drive, elevator, wrist));
      // new JoystickButton(apacController, 2).onTrue(dynamicScore);
      // new JoystickButton(apacController, 3).onTrue(dynamicScore);
      // new JoystickButton(apacController, 4).onTrue(dynamicScore);
      // Cancel button

      controller
          .povDown()
          .onTrue(
              Commands.runOnce(CommandScheduler.getInstance()::cancelAll)
                  .andThen(
                      Commands.parallel(
                          WristCommands.Stowed(wrist), ElevatorCommands.Down(elevator))));
      // for testing elevator/wrist
      new JoystickButton(apacController, 5).onTrue(WristCommands.Stowed(wrist));
      new JoystickButton(apacController, 6).onTrue(WristCommands.AlgaeIntake(wrist));
      new JoystickButton(apacController, 7).onTrue(WristCommands.TestWrist(wrist));
      new JoystickButton(apacController, 12).onTrue(ElevatorCommands.Down(elevator));
      new JoystickButton(apacController, 13).onTrue(ElevatorCommands.L3Score(elevator));

      // Coral intake sequence and simulation helper
      if (coralIntake != null && endEffector != null) {
        new JoystickButton(apacController, 1)
            .toggleOnTrue(
                IntakeCommands.intakeCoral(
                        coralIntake,
                        endEffector,
                        elevator,
                        coralSensor,
                        SubsystemConstants.DEFAULT_CORAL_INTAKE_SPEED,
                        SubsystemConstants.DEFAULT_END_EFFECTOR_SPEED)
                    .withName("Intake Coral"));

        // Simulate sensor trip (toggles SmartDashboard Sim/CoralDetected briefly)
        new JoystickButton(apacController, 2)
            .onTrue(IntakeCommands.simulateCoralDetectionPulse(coralSensor));
      }
    }
  }

  private int getSelectedReefLevel() {
    // Round and clamp to valid levels [1,4]
    int lvl = (int) Math.round(ReefLevel.get());
    if (lvl < 1) return 1;
    if (lvl > 4) return 4;
    return lvl;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Pose2d getStartingPoseForSelectedAuto() {
    Command selected = autoChooser.get();
    return FieldConstants.isBlueAlliance()
        ? autoStartingPosesBlue.get(selected)
        : autoStartingPosesRed.get(selected);
  }

  /* Returns the manually selected starting pose. */
  private Pose2d getManualStartingPose() {
    StartPose selected = startPoseChooser.get();
    return FieldConstants.isBlueAlliance()
        ? manualStartingPosesBlue.get(selected)
        : manualStartingPosesRed.get(selected);
  }

  /* Applies the manually selected starting pose. Something about this is not working*/
  public void applyManualStartingPose() {
    Pose2d pose = getManualStartingPose();
    if (pose != null) {
      drive.setPose(pose);
    }
  }

  /**
   * Applies the starting pose either from the selected autonomous routine or the manual chooser. I
   * don't know why manual starting pose doesn't work currently
   */
  public void applySelectedStartingPose() {
    Pose2d pose = getStartingPoseForSelectedAuto();
    if (pose == null) {
      pose = getManualStartingPose();
    }
    if (pose != null) {
      drive.setPose(pose);
    }
  }
}
