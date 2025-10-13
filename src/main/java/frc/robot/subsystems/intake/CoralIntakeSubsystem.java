package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemConstants;

/**
 * Coral intake subsystem using a CTRE Minion motor controlled by a Talon FXS. Operates in open-loop
 * duty cycle control.
 */
public class CoralIntakeSubsystem extends SubsystemBase {
  private static final TalonFXS intakeMotor =
      new TalonFXS(SubsystemConstants.CoralIntake_ID, SubsystemConstants.CANBUS);
  // private final DutyCycleOut dutyOut = new DutyCycleOut(0.0);

  public CoralIntakeSubsystem() {}

  /** Run intake at a specified voltage while scheduled. Requires the intake subsystem. */
  public static Command runIntake(CoralIntakeSubsystem intake, double volts) {
    return Commands.run(() -> intakeMotor.setVoltage(volts), intake)
        .withName("CoralIntake runVoltage(" + volts + ")");
  }

  /** Stop the intake motor. */
  public void stop() {
    intakeMotor.setVoltage(0);
  }
}
