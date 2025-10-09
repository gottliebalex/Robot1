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
  private static volatile boolean supercycleEnabled = false;

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

  // Supercycle (align to algae SC standoff when scoring coral)
  public static void setSupercycleEnabled(boolean enabled) {
    supercycleEnabled = enabled;
    SmartDashboard.putBoolean("SupercycleEnabled", supercycleEnabled);
  }

  public static boolean isSupercycleEnabled() {
    return supercycleEnabled;
  }

  public static void toggleSupercycle() {
    setSupercycleEnabled(!supercycleEnabled);
  }
}
