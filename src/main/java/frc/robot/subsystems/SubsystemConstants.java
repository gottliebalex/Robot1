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

  public static enum ElevatorPosition {
    Down(Inches.of(8)),
    Intake(Inches.of(16)),
    L1(Inches.of(12)),
    L2(Inches.of(28)),
    L3(Inches.of(40)),
    L4(Inches.of(70)),
    L2Algae(Inches.of(32)),
    L3Algae(Inches.of(45)),
    Barge(Inches.of(85)),
    Zero(Inches.of(0));

    private final Distance distance;

    ElevatorPosition(Distance distance) {
      this.distance = distance;
    }

    public Distance distance() {
      return distance;
    }
  }

  public static enum WristPosition {
    Stowed(Degrees.of(0)),
    AlgaeGroundIntake(Degrees.of(-150)),
    L1Score(Degrees.of(-120)),
    L2Score(Degrees.of(-140)),
    L3Score(Degrees.of(-140)),
    L4Score(Degrees.of(-120)),
    Test(Degrees.of(-180));

    private final Angle angle;

    WristPosition(Angle angle) {
      this.angle = angle;
    }

    public Angle angle() {
      return angle;
    }
  }

  // Tolerances for considering mechanisms "at setpoint"
  public static final Distance ELEVATOR_TOLERANCE = Inches.of(1.0);
  public static final Angle WRIST_TOLERANCE = Degrees.of(5.0);
}
