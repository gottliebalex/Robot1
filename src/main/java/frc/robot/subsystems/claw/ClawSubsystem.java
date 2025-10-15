package frc.robot.subsystems.claw;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemConstants;
import java.util.function.BooleanSupplier;

/** End effector subsystem using a Kraken X60 controlled by a Talon FX. */
public class ClawSubsystem extends SubsystemBase {
  public final TalonFX clawMotor =
      new TalonFX(SubsystemConstants.EndEffector_ID, SubsystemConstants.CANBUS);

  private final TalonFXConfigurator cfg = clawMotor.getConfigurator();

  private final TalonFXConfiguration basecfg = new TalonFXConfiguration();

  private final TorqueCurrentFOC torqueHold = new TorqueCurrentFOC(35.0);

  // algae detection
  private static final double DEFAULT_ALGAE_TRIP_CURRENT_AMPS = 25.0;
  private static final double DEFAULT_ALGAE_DEBOUNCE_S = 0.10;
  private static final double DEFAULT_START_IGNORE_S = 0.15;

  public ClawSubsystem() {
    basecfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    basecfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    basecfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    basecfg.CurrentLimits.SupplyCurrentLimit = 40;
    basecfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    cfg.apply(basecfg);
  }
  // Set motor voltage
  public void setVoltage(double volts) {
    clawMotor.setVoltage(volts);
  }

  // Stop the end effector motor
  public void stop() {
    clawMotor.setVoltage(0);
  }

  public void setOpenLoopRamp(double seconds) {
    OpenLoopRampsConfigs olr = new OpenLoopRampsConfigs();
    olr.VoltageOpenLoopRampPeriod = seconds;
    clawMotor.getConfigurator().apply(olr);
  }

  public void defaulRamp() {
    setOpenLoopRamp(0);
  }

  public static final BooleanSubscriber algaeTripSubscriber =
      NetworkTableInstance.getDefault().getBooleanTopic("/Sim/ForceAlgaeTrip").subscribe(false);

  public BooleanSupplier forceAlgaeTripSupplier() {
    return () -> algaeTripSubscriber.get();
  }

  /** Apply one cycle of torque hold. Intended for use by a default command. */
  public void applyHoldTorque() {
    clawMotor.setControl(torqueHold);
  }

  // /** Run end effector at a voltage for intaking algae */
  // public Command runRollersAlgae(double volts) {
  //   return Commands.run(() -> motor.setVoltage(SubsystemConstants.DEFAULT_ALGAE_INTAKE_SPEED),
  // this)
  //       .finallyDo(() -> stop())
  //       .withName(String.format("EndEffector runRollers", volts));
  // }

  // hold current (amps) for retaining algae
  private static final double DEFAULT_HOLD_CURRENT_AMPS = 6.0;

  /** Hold algae using field‑oriented torque (current) control at the default current. */
  public Command holdAlgae() {
    return holdAlgaeCurrent(DEFAULT_HOLD_CURRENT_AMPS);
  }

  /** Hold algae using field‑oriented torque (current) control, in amps. */
  public Command holdAlgaeCurrent(double amps) {
    return edu.wpi.first.wpilibj2.command.Commands.run(
            () -> clawMotor.setControl(torqueHold.withOutput(amps)), this)
        .withName(String.format("EndEffector holdAlgaeCurrent(%.1fA)", amps));
  }
}
