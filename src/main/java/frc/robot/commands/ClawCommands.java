package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamePiece;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawCommands{

  // private final ClawSubsystem claw;
  // public ClawCommands(ClawSubsystem claw) { this.claw = claw; }


  public static Command setMode(GamePiece.Mode mode) {
    return Commands.runOnce(() -> GamePiece.setMode(mode))
      .withName("SetMode CORAL");
  }

  public static Command setCoralMode() { return setMode(GamePiece.Mode.CORAL); }
  public static Command setAlgaeMode() { return setMode(GamePiece.Mode.ALGAE); }
  public static Command clearMode() { return setMode(GamePiece.Mode.NONE); }


     /**
   * Eject coral by applying a voltage
   *
   *
   * @param volts Voltage to apply to the end effector motor
   */
  public static Command runRollers (
    ClawSubsystem claw,
    double volts,
    double rampS,
    double timeoutS) {
    
    Command runRollers = Commands.startEnd(
      () -> {
      claw.setOpenLoopRamp(rampS);
      claw.clawMotor.setVoltage(volts);
      },
      () -> {
        claw.stop();},
      
      claw 
    ).withName(String.format("EndEffector scoreCoral(%.1fV)", volts));

    if (timeoutS > 0.0) {
      runRollers = runRollers.withTimeout(timeoutS);
    }

    return runRollers;
  }

  public static Command runRollers (
    ClawSubsystem claw,
    SubsystemConstants.ClawVoltages preset) {
    double volts = preset.volts().in(Volts);
    double rampS = preset.rampS().in(Seconds);
    double timeS = preset.timeoutS().in(Seconds);
    return runRollers(claw, volts, rampS, timeS);
  }

  


}