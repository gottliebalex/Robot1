package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemConstants;

/** End effector subsystem using a Kraken X60 controlled by a Talon FX. */
public class EndEffectorSubsystem extends SubsystemBase {
  private final TalonFX motor =
      new TalonFX(SubsystemConstants.EndEffector_ID, SubsystemConstants.CANBUS);
  private final DutyCycleOut dutyOut = new DutyCycleOut(0.0);

  public EndEffectorSubsystem() {}

  /** Run end effector at a duty cycle percent [-1, 1] while scheduled. */
  public Command runDuty(double percent) {
    return Commands.run(() -> motor.setControl(dutyOut.withOutput(percent)), this)
        .withName("EndEffector runDuty(" + percent + ")");
  }

  /** Stop the end effector motor. */
  public void stop() {
    motor.setControl(dutyOut.withOutput(0.0));
  }
}
