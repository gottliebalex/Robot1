package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX armMotor =
      new TalonFX(SubsystemConstants.Wrist_ID, SubsystemConstants.CANBUS);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new
  // SmartMotorControllerTelemetryConfig()
  //          .withMechanismPosition()
  //          .withRotorPosition()
  //          .withMechanismLowerLimit()
  //          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              6, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(1440))
          //      .withSoftLimit(Degrees.of(-360), Degrees.of(360))
          .withGearing(gearing(gearbox(67.407)))
          //      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
          //      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
          .withStatorCurrentLimit(Amps.of(40))
          //      .withVoltageCompensation(Volts.of(12))
          .withMotorInverted(false)
          // .withClosedLoopRampRate(Seconds.of(0.25))
          // .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(.2, .2, .4, .003))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withStartingPosition(Degrees.of(0))
      //      .withClosedLoopControlPeriod(Milliseconds.of(1))
      ;
  private final SmartMotorController motor =
      new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);
  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));

  private ArmConfig m_config =
      new ArmConfig(motor)
          .withLength(Meters.of(0.135))
          .withHardLimit(Degrees.of(-360), Degrees.of(360))
          .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(1))
          .withStartingPosition(Degrees.of(0))
          .withHorizontalZero(Degrees.of(0))
          .withMechanismPositionConfig(robotToMechanism);
  private final Arm arm = new Arm(m_config);

  public static enum WristPosition {
    Stowed(Degrees.of(0)),
    AlgaeGroundIntake(Degrees.of(150)),
    L1Score(Degrees.of(-120)),
    L2Score(Degrees.of(140)),
    L3Score(Degrees.of(140)),
    L4Score(Degrees.of(120)),
    Test(Degrees.of(-180));

    private final Angle angle;

    WristPosition(Angle angle) {
      this.angle = angle;
    }

    public Angle angle() {
      return angle;
    }
  }

  public WristSubsystem() {}

  public void periodic() {
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle) {
    return arm.set(dutycycle);
  }

  public class WristCommands {
    public static Command to(WristSubsystem wrist, WristPosition pos) {
      return Commands.runOnce(() -> wrist.setAngle(pos.angle()), wrist);
    }
  }

  public Command sysId() {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Angle getAngle() {
    return arm.getAngle();
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  public boolean atAngle(Angle target) {
    return Math.abs(getAngle().in(Degrees) - target.in(Degrees))
        <= SubsystemConstants.WRIST_TOLERANCE.in(Degrees);
  }

  public Command waitUntilAtAngle(Angle target) {
    return Commands.waitUntil(() -> atAngle(target));
  }
}
