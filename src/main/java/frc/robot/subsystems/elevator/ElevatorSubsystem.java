package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemConstants;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor =
      new TalonFX(SubsystemConstants.ElevatorLEADER_ID, SubsystemConstants.CANBUS);
  private final TalonFX elevatorFollower =
      new TalonFX(SubsystemConstants.ElevatorFOLLOWER_ID, SubsystemConstants.CANBUS);

  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig =
      new SmartMotorControllerTelemetryConfig()
          .withMechanismPosition()
          .withRotorPosition()
          .withMechanismLowerLimit()
          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(Meters.of(Millimeters.of(5).in(Meters) * 36))
          .withClosedLoopController(
              1.5, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(3))
          .withSimClosedLoopController(
              3.25, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(3.5))
          .withSoftLimit(Meters.of(0), Meters.of(2.5))
          .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(6.8444)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
          //      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(Amps.of(70))
          .withMotorInverted(false)
          .withFeedforward(new ElevatorFeedforward(0.2, 0.3, 0.78, 0.001))
          .withSimFeedforward(new ElevatorFeedforward(0.0, 0.198, 0.89, 0.0065))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withFollowers(Pair.of(elevatorFollower, true))
          .withStartingPosition(Inches.of(0))
      // .withClosedLoopControlPeriod(Milliseconds.of(1))
      ;

  private final SmartMotorController motor =
      new TalonFXWrapper(elevatorMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

  private final MechanismPositionConfig m_robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.2)));
  private ElevatorConfig m_config =
      new ElevatorConfig(motor)
          .withStartingHeight(Meters.of(0))
          .withHardLimits(Meters.of(0), Meters.of(2.5))
          .withTelemetry("Elevator", TelemetryVerbosity.MID)
          .withMechanismPositionConfig(m_robotToMechanism)
          .withMass(Pounds.of(16));
  private final Elevator m_elevator = new Elevator(m_config);

  public ElevatorSubsystem() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        25,
        elevatorMotor.getPosition(),
        elevatorMotor.getVelocity(),
        elevatorMotor.getClosedLoopReference());
    //     elevatorMotor.optimizeBusUtilization();

  }

  public void periodic() {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic() {
    m_elevator.simIterate();
  }

  public Command setHeight(Distance height) {
    return m_elevator.setHeight(height);
  }

  public Distance getHeight() {
    return m_elevator.getHeight();
  }

  public boolean atHeight(Distance target) {
    return Math.abs(getHeight().in(Meters) - target.in(Meters))
        <= SubsystemConstants.ELEVATOR_TOLERANCE.in(Meters);
  }

  public Command waitUntilAtHeight(Distance target) {
    return edu.wpi.first.wpilibj2.command.Commands.waitUntil(() -> atHeight(target));
  }

  public Command sysId() {
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }
}
