package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/** Global game piece state for simple mode selection (e.g., CORAL vs ALGAE). */
public final class GamePieceState {
  public GamePieceState() {}

  public enum Mode {
    NONE,
    CORAL,
    ALGAE
  }

  private static volatile Mode current = Mode.NONE;

  // Logged + networked toggle
  private static final LoggedNetworkBoolean supercycleToggle =
      new LoggedNetworkBoolean("Autopilot/Supercycle", false);
  // Networked copy of the current game piece mode for Elastic UI (distinct from log output)
  private static final LoggedNetworkString GamePieceMode =
      new LoggedNetworkString("UI/GamePieceMode", Mode.NONE.name());

  private static final List<Consumer<Mode>> listeners = new ArrayList<>();

  public static void setMode(Mode mode) {
    current = mode;
    Logger.recordOutput("GamePiece/Mode", current.name());
    GamePieceMode.set(current.name());
    // Notify listeners of mode change
    for (var listener : listeners) {
      try {
        listener.accept(current);
      } catch (Exception ignored) {
      }
    }
  }

  public static Mode getMode() {
    return current;
  }

  public static void onModeChange(Consumer<Mode> listener) {
    listeners.add(listener);
  }

  public static boolean isCoral() {
    return current == Mode.CORAL;
  }

  public static boolean isAlgae() {
    return current == Mode.ALGAE;
  }

  // Supercycle (align to algae SC standoff when scoring coral)
  public static void setSupercycleEnabled(boolean enabled) {
    supercycleToggle.set(enabled);
  }

  public static boolean isSupercycleEnabled() {
    return supercycleToggle.get();
  }

  public static void toggleSupercycle() {
    setSupercycleEnabled(!isSupercycleEnabled());
  }

  private final LoggedNetworkBoolean netReady = new LoggedNetworkBoolean("State/NetReady", false);

  public boolean isNetReady() {
    return netReady.get();
  } // read current value

  public void setNetReady(boolean value) {
    netReady.set(value);
  } // write the value and log it

  public void toggleNetReady() {
    netReady.set(!netReady.get());
  } // to toggle reads current value and set the inverse

  public void reset() {
    netReady.set(false);
  } // reset to known baseline
}
