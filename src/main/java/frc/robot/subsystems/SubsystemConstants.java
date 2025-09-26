package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SubsystemConstants {
  public static final String CANBUS = "rio";
  public static final int ElevatorLEADER_ID = 15;
  public static final int ElevatorFOLLOWER_ID = 14;
  public static final int Wrist_ID = 18;

  // Tolerances for considering mechanisms "at setpoint"
  public static final Distance ELEVATOR_TOLERANCE = Inches.of(1.0);
  public static final Angle WRIST_TOLERANCE = Degrees.of(5.0);
}
