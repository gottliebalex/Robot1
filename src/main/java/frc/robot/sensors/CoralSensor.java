package frc.robot.sensors;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemConstants;
import java.util.Map;

/**
 * Wrapper for the Redux Phosphorus CANandcolor sensor with sim fallback.
 *
 * <p>Real hardware: Uses proximity from {@link Canandcolor#getProximity()} and considers "tripped"
 * when proximity is at or below {@code CORAL_SENSOR_PROX_TRIP}. Proximity range is [0..1] and
 * increases as objects move away (per vendor docs).
 *
 * <p>Simulation: Reads boolean from SmartDashboard key {@code Sim/CoralDetected}. Use the "Simulate
 * Coral Detection" command to pulse it.
 */
public class CoralSensor {
  private static final String kSimKey = "Sim/SimCoralTrip";
  private static final String kThresholdKey = "Redux/CoralThreshold";

  private final boolean useHardware;
  private final Canandcolor device; // null in sim
  private volatile double tripThreshold = SubsystemConstants.CORAL_SENSOR_PROX_TRIP;
  // Shuffleboard entries for live tuning/telemetry
  private final ShuffleboardTab tab;
  private final GenericEntry thresholdEntry;
  private final GenericEntry proxEntry;
  private final GenericEntry detectedEntry;

  /** Create a simulated sensor (no hardware), controlled via SmartDashboard. */
  public CoralSensor() {
    this.useHardware = false;
    this.device = null;
    this.tab = Shuffleboard.getTab("Coral Sensor");
    this.thresholdEntry =
        tab.add("Prox Trip", tripThreshold)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 1.0, "block increment", 0.01))
            .getEntry();
    this.proxEntry = tab.add("Proximity", 0.0).getEntry();
    this.detectedEntry = tab.add("Detected", false).getEntry();
    SmartDashboard.putNumber(kThresholdKey, tripThreshold);
  }

  /** Create a real sensor instance on the RIO CAN bus. */
  public CoralSensor(int canId) {
    this.useHardware = RobotBase.isReal();
    this.device = useHardware ? new Canandcolor(canId) : null;
    this.tab = Shuffleboard.getTab("Coral Sensor");
    this.thresholdEntry =
        tab.add("Prox Trip", tripThreshold)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 1.0, "block increment", 0.01))
            .getEntry();
    this.proxEntry = tab.add("Proximity", 0.0).getEntry();
    this.detectedEntry = tab.add("Detected", false).getEntry();
    SmartDashboard.putNumber(kThresholdKey, tripThreshold);
    if (useHardware && this.device != null) {
      // Configure proximity frame at 20ms for responsive detection
      this.device.setSettings(
          new CanandcolorSettings()
              .setProximityFramePeriod(SubsystemConstants.CORAL_SENSOR_PROX_PERIOD_S));
    }
  }

  /** Returns true when the sensor is tripped (coral detected). */
  public boolean isTripped() {
    // Allow live tuning via Shuffleboard; keep SmartDashboard key in sync
    double uiThreshold =
        thresholdEntry != null ? thresholdEntry.getDouble(tripThreshold) : tripThreshold;
    double effectiveThreshold = (Constants.TUNING_ENABLED) ? uiThreshold : tripThreshold;
    SmartDashboard.putNumber(kThresholdKey, effectiveThreshold);

    if (useHardware && device != null) {
      double prox = device.getProximity();
      boolean tripped = prox <= effectiveThreshold;
      if (proxEntry != null) proxEntry.setDouble(prox);
      if (detectedEntry != null) detectedEntry.setBoolean(tripped);
      SmartDashboard.putNumber("Redux/CoralProximity", prox);
      SmartDashboard.putBoolean("Redux/CoralDetected", tripped);
      return tripped;
    }
    boolean sim = SmartDashboard.getBoolean(kSimKey, false);
    if (proxEntry != null)
      proxEntry.setDouble(sim ? effectiveThreshold * 0.5 : effectiveThreshold + 0.1);
    if (detectedEntry != null) detectedEntry.setBoolean(sim);
    SmartDashboard.putBoolean("Redux/CoralDetected", sim);
    return sim;
  }

  /** Adjust trip threshold (<= threshold considered detected). */
  public void setTripThreshold(double threshold) {
    this.tripThreshold = threshold;
    if (thresholdEntry != null) thresholdEntry.setDouble(threshold);
    SmartDashboard.putNumber(kThresholdKey, threshold);
  }

  /** Set or clear the simulated detection value. */
  public void setSimTripped(boolean value) {
    SmartDashboard.putBoolean(kSimKey, value);
  }
}
