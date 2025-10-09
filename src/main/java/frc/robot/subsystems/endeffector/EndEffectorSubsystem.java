package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GamePiece;
import frc.robot.subsystems.SubsystemConstants;

/** End effector subsystem using a Kraken X60 controlled by a Talon FX. */
public class EndEffectorSubsystem extends SubsystemBase {
  private final TalonFX motor =
      new TalonFX(SubsystemConstants.EndEffector_ID, SubsystemConstants.CANBUS);
  private final DutyCycleOut dutyOut = new DutyCycleOut(0.0);
  private final TorqueCurrentFOC torqueHold = new TorqueCurrentFOC(0.0);

  // algae detection
  private static final double DEFAULT_ALGAE_TRIP_CURRENT_AMPS = 25.0;
  private static final double DEFAULT_ALGAE_DEBOUNCE_S = 0.10;
  private static final double DEFAULT_START_IGNORE_S = 0.15;
  private volatile boolean forceTrip;
  private final BooleanSubscriber algaeTripSubscriber =
    NetworkTableInstance.getDefault().getBooleanTopic("/Sim/ForceAlgaeTrip")
        .subscribe(false);

  public EndEffectorSubsystem() {}

  /** Run end effector at a voltage for intaking algae */
  public Command runRollers(double volts) {
    return Commands.run(() -> motor.setVoltage(SubsystemConstants.DEFAULT_ALGAE_INTAKE_SPEED), this)
        .finallyDo(() -> stop())
        .withName(String.format("EndEffector runRollers", volts));
  }
  /** Stop the end effector motor. */
  public void stop() {
    motor.setVoltage(0);
  }

  // Default hold current (amps) for retaining algae; tune as needed
  private static final double DEFAULT_HOLD_CURRENT_AMPS = 6.0;

  /** Hold algae using field‑oriented torque (current) control at the default current. */
  public Command holdAlgae() {
    return holdAlgaeCurrent(DEFAULT_HOLD_CURRENT_AMPS);
  }

  /** Hold algae using field‑oriented torque (current) control, in amps. */
  public Command holdAlgaeCurrent(double amps) {
    return edu.wpi.first.wpilibj2.command.Commands.run(
            () -> motor.setControl(torqueHold.withOutput(amps)), this)
        .withName(String.format("EndEffector holdAlgaeCurrent(%.1fA)", amps));
  }

  /**
   * Eject coral by applying a fixed voltage while scheduled. Caller controls duration
   * (e.g., using {@code withTimeout(seconds)} at the call site).
   *
   * @param volts Voltage to apply to the end effector motor (positive/negative per wiring)
   */
  public Command scoreCoral(double volts) {
    return Commands.run(() -> motor.setVoltage(volts), this)
        .finallyDo(() -> stop())
        .withName(String.format("EndEffector scoreCoral(%.1fV)", volts));
  }

  /**
   * Intake algae at a given duty cycle until a current spike is detected for a sustained period. On
   * detection, stops the motor and sets the global game piece mode to ALGAE.
   */
  public Command intakeAlgae(double volts) {
    return intakeAlgae(volts, DEFAULT_ALGAE_TRIP_CURRENT_AMPS, 
    DEFAULT_ALGAE_DEBOUNCE_S, algaeTripSubscriber);
  }

  /**
   * Intake algae with current trip and debounce
   *
   * @param volts 
   * @param tripCurrentAmps Current threshold in amps considered as "algae engaged"
   * @param debounceSeconds Time that current must remain above threshold to count as engaged
   */
  public Command intakeAlgae(
    double volts, 
    double tripCurrentAmps, 
    double debounceSeconds,
    BooleanSupplier forceTrip) {

      Debouncer debouncer = new Debouncer(debounceSeconds);
      Timer ignoreTimer = new Timer();
      ignoreTimer.restart();

      return Commands
          .startEnd(() -> motor.setVoltage(volts), () -> motor.stopMotor(), this)
          .until(
              () -> {
                // Refresh the stator current once per loop, then read
                var stator = motor.getStatorCurrent();
                BaseStatusSignal.refreshAll(stator);
                double amps = stator.getValueAsDouble();

                boolean aboveThreshhold =
                    ignoreTimer.hasElapsed(DEFAULT_START_IGNORE_S) 
                    && amps >= tripCurrentAmps;
                return debouncer.calculate(aboveThreshhold) || forceTrip.getAsBoolean();
              })
          .andThen(holdAlgae())
          .andThen(Commands.runOnce(() -> GamePiece.setMode(GamePiece.Mode.ALGAE)))
          .withName(
              String.format(
                  "EndEffector intakeAlgae (V=%.1f, trip=%.1fA, debounce=%.2fs)",
                  volts, tripCurrentAmps, debounceSeconds));
    }
}
