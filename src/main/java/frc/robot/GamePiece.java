package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** Global game piece state for simple mode selection (e.g., CORAL vs ALGAE). */
public final class GamePiece {
  private GamePiece() {}

  public enum Mode {
    NONE,
    CORAL,
    ALGAE
  }

  private static volatile Mode current = Mode.NONE;

  // Logged + networked toggle
  private static final LoggedNetworkBoolean supercycleToggle =
      new LoggedNetworkBoolean("Autopilot/Supercycle", false);

  public static void setMode(Mode mode) {
    current = mode;
    Logger.recordOutput("GamePiece/Mode", current.name());
  }

  public static Mode getMode() {
    return current;
  }

  public static boolean isCoral() {
    return current == Mode.CORAL;
  }

  // Supercycle (align to algae SC standoff when scoring coral)
  public static void setSupercycleEnabled(boolean enabled) { supercycleToggle.set(enabled); }

  public static boolean isSupercycleEnabled() { return supercycleToggle.get(); }

  public static void toggleSupercycle() {
    setSupercycleEnabled(!isSupercycleEnabled());
  }
}
