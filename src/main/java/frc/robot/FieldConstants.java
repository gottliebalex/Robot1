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

  // Cached alliance to avoid repeated DriverStation lookups on hot paths
  private static volatile Alliance ALLIANCE_CACHE =
      DriverStation.getAlliance().orElse(Alliance.Blue);

  public static Alliance getAllianceCached() {
    return ALLIANCE_CACHE;
  }

  public static void setAllianceCached(Alliance alliance) {
    ALLIANCE_CACHE = alliance;
    // Notify nested classes that depend on alliance selection
    Reef.onAllianceUpdated(ALLIANCE_CACHE);
  }

  public static void refreshAllianceCache() {
    setAllianceCached(DriverStation.getAlliance().orElse(Alliance.Blue));
  }

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
      RIGHT,
      CENTER
    }

    /** Scoring levels for precomputation using EnumMap. */
    public enum Level {
      L1,
      L2,
      L3,
      L4;

      public static Level fromInt(int level) {
        return switch (level) {
          case 1 -> L1;
          case 2 -> L2;
          case 3 -> L3;
          default -> L4;
        };
      }
    }

    public static final Translation2d CENTER_BLUE = new Translation2d(4.4893, 4.0259);

    /** Distance from reef center to the face plane center (BLUE ref). */
    public static final double FACE_RADIUS = 0.83175;

    /** Orientation of each face (normal pointing away from reef center), BLUE ref. */
    private static final EnumMap<Branch, Rotation2d> FACE_NORMALS_BLUE =
        new EnumMap<>(Branch.class);
    /** Center point of each face (BLUE ref). */
    private static final EnumMap<Branch, Translation2d> FACE_CENTERS_BLUE =
        new EnumMap<>(Branch.class);

    /** BLUE face poses (center + outward normal). */
    private static final EnumMap<Branch, Pose2d> FACE_POSES_BLUE = new EnumMap<>(Branch.class);
    /** RED face poses (pre-flipped from BLUE). */
    private static final EnumMap<Branch, Pose2d> FACE_POSES_RED = new EnumMap<>(Branch.class);

    // Precomputed scoring poses (no side) for BLUE and RED
    private static final EnumMap<Branch, EnumMap<Level, Pose2d>> SCORING_NO_SIDE_BLUE =
        new EnumMap<>(Branch.class);
    private static final EnumMap<Branch, EnumMap<Level, Pose2d>> SCORING_NO_SIDE_RED =
        new EnumMap<>(Branch.class);
    // Precomputed scoring poses (with side) for BLUE and RED
    private static final EnumMap<Branch, EnumMap<Level, EnumMap<PipeSide, Pose2d>>>
        SCORING_SIDE_BLUE = new EnumMap<>(Branch.class);
    private static final EnumMap<Branch, EnumMap<Level, EnumMap<PipeSide, Pose2d>>>
        SCORING_SIDE_RED = new EnumMap<>(Branch.class);

    // CURRENT views that point to the active alliance maps. Reassigned when alliance changes
    private static Map<Branch, Pose2d> CURRENT_FACE_POSES = FACE_POSES_BLUE;
    private static Map<Branch, EnumMap<Level, Pose2d>> CURRENT_SCORING_NO_SIDE =
        SCORING_NO_SIDE_BLUE;
    private static Map<Branch, EnumMap<Level, EnumMap<PipeSide, Pose2d>>> CURRENT_SCORING_SIDE =
        SCORING_SIDE_BLUE;

    /** Standoff per level, negative moves robot *toward* the reef along the face normal. */
    public static final double STANDOFF_L1 = 0.4;

    public static final double STANDOFF_L2 = 0.45;
    public static final double STANDOFF_L3 = STANDOFF_L2;
    public static final double STANDOFF_L4 = 0.65;

    static {
      // Define hexagon around +X and go clockwise every 60°
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

    // Precompute face poses and scoring targets for BLUE and RED once
    static {
      // Face poses
      for (var e : FACE_CENTERS_BLUE.entrySet()) {
        Branch br = e.getKey();
        Pose2d bluePose = new Pose2d(e.getValue(), FACE_NORMALS_BLUE.get(br));
        FACE_POSES_BLUE.put(br, bluePose);
        FACE_POSES_RED.put(br, allianceFlip(bluePose));
      }

      // Scoring poses for all branches, all levels
      for (Branch br : Branch.values()) {
        SCORING_NO_SIDE_BLUE.put(br, new EnumMap<>(Level.class));
        SCORING_NO_SIDE_RED.put(br, new EnumMap<>(Level.class));
        SCORING_SIDE_BLUE.put(br, new EnumMap<>(Level.class));
        SCORING_SIDE_RED.put(br, new EnumMap<>(Level.class));

        Pose2d blueBase = FACE_POSES_BLUE.get(br);
        for (Level lvl : Level.values()) {
          double standoff =
              switch (lvl) {
                case L1 -> STANDOFF_L1;
                case L2 -> STANDOFF_L2;
                case L3 -> STANDOFF_L3;
                case L4 -> STANDOFF_L4;
              };
          Translation2d normalOffset =
              new Translation2d(standoff, 0).rotateBy(blueBase.getRotation());
          Rotation2d inward = blueBase.getRotation();

          // No-side center target
          Pose2d blueNoSide = new Pose2d(blueBase.getTranslation().plus(normalOffset), inward);
          SCORING_NO_SIDE_BLUE.get(br).put(lvl, blueNoSide);
          SCORING_NO_SIDE_RED.get(br).put(lvl, allianceFlip(blueNoSide));

          // Side-specific
          EnumMap<PipeSide, Pose2d> blueSideMap = new EnumMap<>(PipeSide.class);
          EnumMap<PipeSide, Pose2d> redSideMap = new EnumMap<>(PipeSide.class);
          double halfSpacingM = inchesToMeters(12.938) / 2.0;
          Rotation2d leftDir = inward.plus(Rotation2d.fromDegrees(90));

          // CENTER (same as no-side)
          blueSideMap.put(PipeSide.CENTER, blueNoSide);
          redSideMap.put(PipeSide.CENTER, allianceFlip(blueNoSide));
          
          // LEFT pipe BLUE, LEFT pipe RED
          Translation2d blueleftOffset = new Translation2d(-halfSpacingM, 0).rotateBy(leftDir);

          Pose2d blueLeft =
              new Pose2d(blueBase.getTranslation().plus(normalOffset).plus(blueleftOffset), inward);
          blueSideMap.put(PipeSide.LEFT, blueLeft);

          Pose2d redBase = allianceFlip(blueBase);

          Pose2d redLeft =
              redBase.plus(
                  new Transform2d(new Translation2d(standoff, -halfSpacingM), new Rotation2d()));

          redSideMap.put(PipeSide.LEFT, redLeft);

          // RIGHT pipe BLUE, RIGHT pipe RED
          Translation2d bluerightOffset = new Translation2d(+halfSpacingM, 0).rotateBy(leftDir);

          Pose2d blueRight =
              new Pose2d(
                  blueBase.getTranslation().plus(normalOffset).plus(bluerightOffset), inward);
          blueSideMap.put(PipeSide.RIGHT, blueRight);

          Pose2d redRight =
              redBase.plus(
                  new Transform2d(new Translation2d(standoff, +halfSpacingM), new Rotation2d()));

          redSideMap.put(PipeSide.RIGHT, redRight);

          SCORING_SIDE_BLUE.get(br).put(lvl, blueSideMap);
          SCORING_SIDE_RED.get(br).put(lvl, redSideMap);
        }
      }

      // Initialize current views to the cached alliance once
      onAllianceUpdated(FieldConstants.getAllianceCached());
    }

    /** Returns the BLUE-reference face-center pose (position + facing) for a branch. */
    public static Pose2d blueFacePose(Branch branch) {
      return FACE_POSES_BLUE.get(branch);
    }

    public static Pose2d scoringPose(Branch branch, int level) {
      if (CURRENT_SCORING_NO_SIDE != null) {
        Level lvl = Level.fromInt(level);
        return CURRENT_SCORING_NO_SIDE.get(branch).get(lvl);
      }
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

      // Flip for RED
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      return alliance == Alliance.Red ? allianceFlip(blueWithOffsetInward) : blueWithOffsetInward;
      // we should look into using alliancefliputil
    }
    /*
    Scoring pose offset to the selected pipe side (left/right) relative to the face center. The
    offset is applied in the BLUE frame and then alliance-flipped if needed.
    */
    public static Pose2d scoringPose(Branch branch, int level, PipeSide side) {
      if (CURRENT_SCORING_SIDE != null) {
        Level lvl = Level.fromInt(level);
        return CURRENT_SCORING_SIDE.get(branch).get(lvl).get(side);
      }
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
      Rotation2d leftDir = inward.plus(Rotation2d.fromDegrees(90));
      Translation2d lateralOffset;
      if (side == PipeSide.CENTER) {
        lateralOffset = new Translation2d();
      } else {
        double sign = (side == PipeSide.LEFT) ? +1.0 : -1.0;
        lateralOffset = new Translation2d(sign * halfSpacingM, 0).rotateBy(leftDir);
      }

      Pose2d blueTarget =
          new Pose2d(blue.getTranslation().plus(normalOffset).plus(lateralOffset), inward);

      // Alliance-aware flip
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      return alliance == Alliance.Red ? allianceFlip(blueTarget) : blueTarget;
    }

    /*Finds the nearest reef branch (by face center) to a given field pose
     */
    public static Branch nearestBranch(Pose2d robotPose) {
      if (CURRENT_FACE_POSES != null) {
        Branch best = Branch.A;
        double bestDist = Double.POSITIVE_INFINITY;
        for (var e : CURRENT_FACE_POSES.entrySet()) {
          Translation2d fc = e.getValue().getTranslation();
          double d = robotPose.getTranslation().getDistance(fc);
          if (d < bestDist) {
            bestDist = d;
            best = e.getKey();
          }
        }
        return best;
      }
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

    /* Updates current cached maps when alliance changes
    (I don't see when that would happen but why not put it in.). */
    static void onAllianceUpdated(Alliance alliance) {
      boolean isRed = alliance == Alliance.Red;
      CURRENT_FACE_POSES = isRed ? FACE_POSES_RED : FACE_POSES_BLUE;
      CURRENT_SCORING_NO_SIDE = isRed ? SCORING_NO_SIDE_RED : SCORING_NO_SIDE_BLUE;
      CURRENT_SCORING_SIDE = isRed ? SCORING_SIDE_RED : SCORING_SIDE_BLUE;
    }

    private static Pose2d allianceFlip(Pose2d bluePose) {
      // Mirror across the field X-length centerline (x -> L - x) and reflect heading.
      // For a reflection across the vertical centerline, the heading transforms as (pi - theta).
      double x = (FIELD_LENGTH) - bluePose.getX();
      double y = bluePose.getY();
      Rotation2d rot = Rotation2d.fromRadians(Math.PI).minus(bluePose.getRotation());
      return new Pose2d(x, y, rot);
    }
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
    return getAllianceCached() == DriverStation.Alliance.Blue;
  }

  public static final Pose2d LEFT_STARTING_POSE_BLUE =
      new Pose2d(7.07, 7, Rotation2d.fromDegrees(0));
  public static final Pose2d LEFT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(180));

  public static final Pose2d RIGHT_STARTING_POSE_BLUE =
      new Pose2d(
          LEFT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(0));
  public static final Pose2d RIGHT_STARTING_POSE_RED =
      new Pose2d(
          FIELD_LENGTH - RIGHT_STARTING_POSE_BLUE.getX(),
          FIELD_HEIGHT - RIGHT_STARTING_POSE_BLUE.getY(),
          Rotation2d.fromDegrees(180));
}
