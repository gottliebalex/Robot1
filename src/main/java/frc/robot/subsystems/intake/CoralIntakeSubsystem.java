package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
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
  private final TalonFXS motor =
      new TalonFXS(SubsystemConstants.CoralIntake_ID, SubsystemConstants.CANBUS);
  private final DutyCycleOut dutyOut = new DutyCycleOut(0.0);

  public CoralIntakeSubsystem() {}

  /** Run intake at a duty cycle percent [-1, 1] while scheduled. */
  public Command runDuty(double percent) {
    return Commands.run(() -> motor.setControl(dutyOut.withOutput(percent)), this)
        .withName("CoralIntake runDuty(" + percent + ")");
  }

  /** Stop the intake motor. */
  public void stop() {
    motor.setControl(dutyOut.withOutput(0.0));
  }
}
