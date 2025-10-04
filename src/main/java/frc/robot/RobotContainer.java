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
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.WristCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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

    Command startJThenJStation = Autos.choreoStartJThenJStation(drive);
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

    // Left/Right bumpers: align to nearest face LEFT/RIGHT pipe at selected level while held
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.alignToNearestAllianceReefFace(
                drive, () -> getSelectedReefLevel(), PipeSide.LEFT));
    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.alignToNearestAllianceReefFace(
                drive, () -> getSelectedReefLevel(), PipeSide.RIGHT));

    // Center button: align to center pipe (no lateral offset) at selected level while held
    //mostly here for testing but we could also make this button do algae intaking from the reef
    controller
        .y()
        .whileTrue(
            DriveCommands.alignToNearestAllianceReefFace(
                drive, () -> getSelectedReefLevel(), PipeSide.CENTER));

    if (elevator != null && wrist != null) {
      // the scoring commands are very much AI generated without much (any validation) so keep that in mind
      // Scoring commands trigger once on press and finish when done
      Command l2 = ScoreCommands.scoreL2(drive, elevator, wrist);
      Command l3 = ScoreCommands.scoreL3(drive, elevator, wrist);
      Command l4 = ScoreCommands.scoreL4(drive, elevator, wrist);
      new JoystickButton(apacController, 2).onTrue(l2);
      new JoystickButton(apacController, 3).onTrue(l3);
      new JoystickButton(apacController, 4).onTrue(l4);
      // Cancel button
      new JoystickButton(apacController, 1)
          .onTrue(
              Commands.runOnce(
                      () -> {
                        CommandScheduler.getInstance().cancel(l2);
                        CommandScheduler.getInstance().cancel(l3);
                        CommandScheduler.getInstance().cancel(l4);
                      })
                  .andThen(
                      Commands.parallel(
                          WristCommands.Stowed(wrist), ElevatorCommands.Down(elevator))));
      // for testing elevator/wrist
      new JoystickButton(apacController, 5).onTrue(WristCommands.Stowed(wrist));
      new JoystickButton(apacController, 6).onTrue(WristCommands.AlgaeIntake(wrist));
      new JoystickButton(apacController, 7).onTrue(WristCommands.TestWrist(wrist));
      new JoystickButton(apacController, 12).onTrue(ElevatorCommands.Down(elevator));
      new JoystickButton(apacController, 13).onTrue(ElevatorCommands.L3Score(elevator));
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
   * Applies the starting pose either from the selected autonomous routine or the manual chooser.
   I don't know why manual starting pose doesn't work currently*/
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
