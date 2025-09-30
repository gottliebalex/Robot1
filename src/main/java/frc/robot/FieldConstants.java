package frc.robot;

import static edu.wpi.first.math.util.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.EnumMap;
import java.util.Map;

public class FieldConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
  }

  public static final double FIELD_HEIGHT = 8.0518;
  public static final double FIELD_LENGTH = 17.548249;

  public static final class Reef {
    /** Label the 6 reef faces clockwise starting at +X (adjust order to your liking). */
    public enum Branch {
      A,
      B,
      C,
      D,
      E,
      F
    }

    /** Select which pipe on a face to align to when scoring. */
    public enum PipeSide {
      LEFT,
      RIGHT
    }

    public static final Translation2d CENTER_BLUE = new Translation2d(4.4893, 4.0259);

    /** Distance from reef center to the face plane center (BLUE ref). */
    public static final double FACE_RADIUS = 0.83175;

    /** Orientation of each face (normal pointing *away* from reef center). */
    private static final Map<Branch, Rotation2d> FACE_NORMALS_BLUE = new EnumMap<>(Branch.class);
    /** Center point of each face (BLUE ref). */
    private static final Map<Branch, Translation2d> FACE_CENTERS_BLUE = new EnumMap<>(Branch.class);

    /** Standoff per level, negative moves robot *toward* the reef along the face normal. */
    public static final double STANDOFF_L1 = 0.35;

    public static final double STANDOFF_L2 = 0.45;
    public static final double STANDOFF_L3 = 0.55;
    public static final double STANDOFF_L4 = 0.65;

    static {
      // Define a hexagon around +X and go clockwise every 60°
      Rotation2d[] normals = {
        Rotation2d.fromDegrees(0), // A
        Rotation2d.fromDegrees(-60), // B
        Rotation2d.fromDegrees(-120), // C
        Rotation2d.fromDegrees(180), // D
        Rotation2d.fromDegrees(120), // E
        Rotation2d.fromDegrees(60) // F
      };
      Branch[] order = {Branch.A, Branch.B, Branch.C, Branch.D, Branch.E, Branch.F};
      for (int i = 0; i < order.length; i++) {
        FACE_NORMALS_BLUE.put(order[i], normals[i]);
        FACE_CENTERS_BLUE.put(
            order[i], CENTER_BLUE.plus(new Translation2d(FACE_RADIUS, 0.0).rotateBy(normals[i])));
      }
    }

    /** Returns the BLUE-reference face-center pose (position + facing) for a branch. */
    public static Pose2d blueFacePose(Branch branch) {
      return new Pose2d(FACE_CENTERS_BLUE.get(branch), FACE_NORMALS_BLUE.get(branch));
    }

    public static Pose2d scoringPose(Branch branch, int level) {
      Pose2d blue = blueFacePose(branch);
      double standoff =
          switch (level) {
            case 1 -> STANDOFF_L1;
            case 2 -> STANDOFF_L2;
            case 3 -> STANDOFF_L3;
            default -> STANDOFF_L4;
          };
      // Place target in front of the face (outside the reef) along the face normal
      Translation2d offset = new Translation2d(standoff, 0).rotateBy(blue.getRotation());
      // Desired scoring orientation faces the reef (inward: normal + 180°)
      Pose2d blueWithOffsetInward =
          new Pose2d(
              blue.getTranslation().plus(offset),
              blue.getRotation().plus(Rotation2d.fromDegrees(180)));

      // Flip for RED if needed
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      return alliance == Alliance.Red ? allianceFlip(blueWithOffsetInward) : blueWithOffsetInward;
      // If you have AllianceFlipUtil.apply(blueWithOffsetInward), use that instead of
      // allianceFlip(...)
    }

    /**
     * Scoring pose offset to the selected pipe side (left/right) relative to the face center. The
     * offset is applied in the BLUE frame and then alliance-flipped if needed.
     */
    public static Pose2d scoringPose(Branch branch, int level, PipeSide side) {
      Pose2d blue = blueFacePose(branch);

      // Standoff along face normal (positive is away from reef)
      double standoff =
          switch (level) {
            case 1 -> STANDOFF_L1;
            case 2 -> STANDOFF_L2;
            case 3 -> STANDOFF_L3;
            default -> STANDOFF_L4;
          };
      Translation2d normalOffset = new Translation2d(standoff, 0).rotateBy(blue.getRotation());

      // Robot should face inward (toward the reef)
      Rotation2d inward = blue.getRotation().plus(Rotation2d.fromDegrees(180));

      // Lateral offset to the chosen pipe: half of the pipe center spacing, left is +90° from
      // inward, right is -90°
      double halfSpacingM = inchesToMeters(12.938) / 2.0;
      double sign = (side == PipeSide.LEFT) ? +1.0 : -1.0;
      Rotation2d leftDir = inward.plus(Rotation2d.fromDegrees(90));
      Translation2d lateralOffset = new Translation2d(sign * halfSpacingM, 0).rotateBy(leftDir);

      Pose2d blueTarget =
          new Pose2d(blue.getTranslation().plus(normalOffset).plus(lateralOffset), inward);

      // Alliance-aware flip
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      return alliance == Alliance.Red ? allianceFlip(blueTarget) : blueTarget;
    }

    /**
     * Finds the nearest reef branch (by face center) to a given field pose for convenience
     * targeting.
     */
    public static Branch nearestBranch(Pose2d robotPose) {
      Branch best = Branch.A;
      double bestDist = Double.POSITIVE_INFINITY;
      for (var e : FACE_CENTERS_BLUE.entrySet()) {
        Translation2d fc = e.getValue();
        double d =
            robotPose
                .getTranslation()
                .getDistance(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? allianceFlip(new Pose2d(fc, new Rotation2d())).getTranslation()
                        : fc);
        if (d < bestDist) {
          bestDist = d;
          best = e.getKey();
        }
      }
      return best;
    }

    private static Pose2d allianceFlip(Pose2d bluePose) {
      // Mirror across the field X-length centerline (x -> L - x) and reflect heading.
      // For a reflection across the vertical centerline, the heading transforms as (pi - theta).
      double x = (FIELD_LENGTH) - bluePose.getX();
      double y = bluePose.getY();
      Rotation2d rot = Rotation2d.fromRadians(Math.PI).minus(bluePose.getRotation());
      return new Pose2d(x, y, rot);
    }
    // Convenience Units helper (so kFieldLength can be stored in meters or feet—your call)
    private static double meters(double value) {
      return value;
    }

    private static double metersToFeet(double meters) {
      return meters;
    } // keep in meters if you already are
  }

  // April tag IDs
  public static final int RED_LEFT_CORAL_STATION = 1;
  public static final int RED_RIGHT_CORAL_STATION = 2;
  public static final int RED_PROCESSOR = 3;
  public static final int RED_RIGHT_NET = 4;
  public static final int RED_LEFT_NET = 5;
  public static final int RED_REEF_LEFT_DRIVER_STATION = 6;
  public static final int RED_REEF_CENTER_DRIVER_STATION = 7;
  public static final int RED_REEF_RIGHT_DRIVER_STATION = 8;
  public static final int RED_REEF_RIGHT_BARGE = 9;
  public static final int RED_REEF_CENTER_BARGE = 10;
  public static final int RED_REEF_LEFT_BARGE = 11;
  public static final int BLUE_RIGHT_CORAL_STATION = 12;
  public static final int BLUE_LEFT_CORAL_STATION = 13;
  public static final int BLUE_LEFT_BARGE = 14;
  public static final int BLUE_RIGHT_BARGE = 15;
  public static final int BLUE_PROCESSOR = 16;
  public static final int BLUE_REEF_RIGHT_DRIVER_STATION = 17;
  public static final int BLUE_REEF_CENTER_DRIVER_STATION = 18;
  public static final int BLUE_REEF_LEFT_DRIVER_STATION = 19;
  public static final int BLUE_REEF_RIGHT_BARGE = 20;
  public static final int BLUE_REEF_CENTER_BARGE = 21;
  public static final int BLUE_REEF_LEFT_BARGE = 22;

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Blue;
  }

  public static final Pose2d LEFT_STARTING_POSE_BLUE =
      new Pose2d(7.067, 6.626, Rotation2d.fromDegrees(180));
  public static final Pose2d LEFT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(-180));

  public static final Pose2d RIGHT_STARTING_POSE_BLUE =
      new Pose2d(
          LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(-180.0));
  public static final Pose2d RIGHT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - RIGHT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - RIGHT_STARTING_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(180.0));
}
