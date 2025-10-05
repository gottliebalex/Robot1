package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Global game piece state for simple mode selection (e.g., CORAL vs ALGAE). */
public final class GamePiece {
  private GamePiece() {}

  public enum Mode {
    NONE,
    CORAL,
    ALGAE
  }

  private static volatile Mode current = Mode.NONE;

  public static void setMode(Mode mode) {
    current = mode;
    SmartDashboard.putString("GamePieceMode", current.name());
  }

  public static Mode getMode() {
    return current;
  }

  public static boolean isCoral() {
    return current == Mode.CORAL;
  }
}
