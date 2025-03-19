// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode CURRENT_MODE = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static final String LOW_BACK_CAMERA_NAME = "back_arducam";
    public static final String LEFT_CAMERA_NAME = "left_arducam";
    public static final String RIGHT_CAMERA_NAME = "right_arducam";

    public static final String LIMELIGHT_NAME = "limelight-phoenix";

    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.2944),
                Units.inchesToMeters(8.9822),
                Units.inchesToMeters(12.125)),
            new Rotation3d(0, 0.0, Units.degreesToRadians(-35)));

    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.2944),
                Units.inchesToMeters(-8.9822),
                Units.inchesToMeters(12.125)),
            new Rotation3d(0, 0.0, Units.degreesToRadians(35)));

    public static final Transform3d LOW_BACK_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.73),
                Units.inchesToMeters(11.286),
                Units.inchesToMeters(7.89)),
            new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(180)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.1;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS =
        new double[] {
          1.0, // Camera 0
          1.0, // Camera 1
          1.0 // Camera 2
        };

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR =
        0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class FieldConstants {
    public static final Pose3d[] ZONE_ALIGN_BLUE_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(21).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(22).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(17).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(18).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(19).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(20).orElse(new Pose3d())
        };
    public static final Pose3d[] ZONE_ALIGN_RED_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(7).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(6).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(11).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(10).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(9).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(8).orElse(new Pose3d())
        };
    public static final double REEF_BUFFER = DriveConstants.DRIVE_BASE_RADIUS + 0.5;
    public static final Transform2d REEF_BUFFER_TRANSFORM =
        new Transform2d(REEF_BUFFER, 0, Rotation2d.k180deg);
    public static final Transform2d REEF_PATH_BUFFER =
        new Transform2d(REEF_BUFFER + 0.1, 0, Rotation2d.k180deg);

    public static final Pose2d BLUE_REEF_CENTER = new Pose2d(4.489323, 4.0259, new Rotation2d());
    public static final Pose2d RED_REEF_CENTER = new Pose2d(13.058902, 4.0259, new Rotation2d());

    public static final double X_LIMIT = 3.25;
    public static final double Y_LIMIT = 3.5;
    public static final double SLOPE = 0.61261261261261;
    public static final Transform2d[] ZONE_TRANSFORMS =
        new Transform2d[] {
          new Transform2d(-X_LIMIT, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(X_LIMIT, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(X_LIMIT, -Y_LIMIT, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, -Y_LIMIT, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(-1.14, 0.68, Rotation2d.kZero),
          new Transform2d(0, 1.32, Rotation2d.kZero),
          new Transform2d(0, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(0, 1.32, Rotation2d.kZero),
          new Transform2d(1.14, 0.68, Rotation2d.kZero),
          new Transform2d(X_LIMIT, SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(1.14, 0.68, Rotation2d.kZero),
          new Transform2d(1.14, -0.68, Rotation2d.kZero),
          new Transform2d(X_LIMIT, -SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(1.14, -0.68, Rotation2d.kZero),
          new Transform2d(0, -1.32, Rotation2d.kZero),
          new Transform2d(0, -Y_LIMIT, Rotation2d.kZero),
          new Transform2d(0, -1.32, Rotation2d.kZero),
          new Transform2d(-1.14, -0.68, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, -SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(-1.14, -0.68, Rotation2d.kZero),
          new Transform2d(-1.14, 0.68, Rotation2d.kZero),
        };

    public static final double BRANCH_STRAFE = Units.inchesToMeters(6.5);
    public static final double BRANCH_BUFFER = DriveConstants.DRIVE_BASE_RADIUS + 0.15;
    public static final Transform2d LEFT_BRANCH =
        new Transform2d(BRANCH_BUFFER, -BRANCH_STRAFE, Rotation2d.k180deg);
    public static final Transform2d RIGHT_BRANCH =
        new Transform2d(BRANCH_BUFFER, BRANCH_STRAFE, Rotation2d.k180deg);

    public static final Pose2d[] BLUE_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(13).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(12).orElse(new Pose3d()).toPose2d(),
        };
    public static final Pose2d[] RED_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(1).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(2).orElse(new Pose3d()).toPose2d(),
        };

    public static final double STATION_BUFFER = DriveConstants.DRIVE_BASE_RADIUS + 0.05;
    public static final Transform2d CENTER_PLAYER_STATION =
        new Transform2d(STATION_BUFFER, 0, Rotation2d.k180deg);

    public static final Transform2d PROCESSOR_BUFFER =
        new Transform2d(DriveConstants.DRIVE_BASE_RADIUS + 0.12, 0, Rotation2d.k180deg);
    public static final Pose2d BLUE_PROCESSOR =
        VisionConstants.TAG_LAYOUT
            .getTagPose(16)
            .orElse(new Pose3d())
            .toPose2d()
            .plus(PROCESSOR_BUFFER);
    public static final Pose2d RED_PROCESSOR =
        VisionConstants.TAG_LAYOUT
            .getTagPose(3)
            .orElse(new Pose3d())
            .toPose2d()
            .plus(PROCESSOR_BUFFER);

    public static final Pose2d[] ALGAE_BLUE_POSES =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(21).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(22).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(17).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(18).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(19).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(20).orElse(new Pose3d()).toPose2d(),
        };
    public static final int[] ALGAE_BLUE_STATES = new int[] {6, 7, 6, 7, 6, 7};
    public static final Pose2d[] ALGAE_RED_POSES =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(7).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(6).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(11).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(10).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(9).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(8).orElse(new Pose3d()).toPose2d(),
        };
    public static final int[] ALGAE_RED_STATES = new int[] {7, 6, 7, 6, 7, 6};
  }

  public static final class AutoConstants {
    public static final PathConstraints CONSTRAINTS =
        new PathConstraints(
            DriveConstants.MAX_LINEAR_SPEED,
            3.0,
            Units.degreesToRadians(720),
            Units.degreesToRadians(1080));

    public static final Constraints LINEAR_CONSTRAINTS =
        new Constraints(DriveConstants.MAX_LINEAR_SPEED, 4);
    public static final Constraints ANGLE_CONSTRAINTS =
        new Constraints(Units.degreesToRadians(720), Units.degreesToRadians(1080));

    public static final Constraints FINE_LINEAR_CONSTRAINTS = new Constraints(2, 0.75);
    public static final Constraints FINE_ANGLE_CONSTRAINTS =
        new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final Transform2d PATHING_BUFFER =
        new Transform2d(DriveConstants.DRIVE_BASE_RADIUS + 0.5, 0, Rotation2d.k180deg);

    public static final Pose3d[] BLUE_REEF_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(21).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(22).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(17).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(18).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(19).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(20).orElse(new Pose3d())
        };
    public static final Pose3d[] RED_REEF_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(10).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(9).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(8).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(7).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(6).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(11).orElse(new Pose3d())
        };

    public static final Pose2d[] PATHING_BLUE_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(13).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(12).orElse(new Pose3d()).toPose2d()
        };

    public static final Pose2d[] PATHING_RED_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(1).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(2).orElse(new Pose3d()).toPose2d()
        };
  }

  public static final class DriveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.941);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(112);
    public static final double ROBOT_MOI = 5.278;
    public static final double WHEEL_COF = 1.2;
    public static final double SLIP_CURRENT = 40;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    // public static final double[] DEV_ENCODER_OFFSETS = {2.888, -2.246 + Math.PI, -2.976, -2.745};
    public static final double[] COMP_ENCODER_OFFSETS = {
      0.815 + Math.PI, 2.481 + Math.PI, 0.82 + Math.PI, -0.7 + Math.PI
    };

    public static final double[] ENCODER_OFFSETS = COMP_ENCODER_OFFSETS;
  }

  public static final class SuperstructureConstants {
    public static final double[] ELEVATOR_STATES = {
      0, 0.122, 0.257, 0.575, 0.975, 1.7, 0.17, 0.59, 0, 0.3
    };
    public static final double[] WRIST_STATES = {
      WristConstants.MAX_ANGLE - 0.2,
      1.222,
      -0.17,
      -0.53,
      -0.53,
      -0.692,
      0.1,
      0.1,
      0,
      WristConstants.MAX_ANGLE - 0.1
    };
    public static final String[] STATE_NAMES = {
      "STOWED",
      "INTAKE",
      "L1",
      "L2",
      "L3",
      "L4",
      "ALGAE LOW",
      "ALGAE HIGH",
      "PROCESSOR/ZERO",
      "SPEED MOVE"
    };
  }

  public static final class ElevatorConstants {
    public static final double GEAR_RATIO = 62.0 / 8.0;
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(25.0);
    public static final double PULLEY_RADIUS = Units.inchesToMeters(1.751) / 2;
    public static final double MIN_HEIGHT = Units.inchesToMeters(10);
    public static final double MAX_EXTENSION = 1.75;
    public static final double MAX_HEIGHT = MIN_HEIGHT + MAX_EXTENSION;
    public static final double MAGIC_NUMBER = PULLEY_RADIUS * 3;
    public static final double CHARACTERIZATION_CUTOFF_HEIGHT = (MAX_HEIGHT - MIN_HEIGHT) / 1.5;
  }

  public static final class ClawConstants {
    public static final double LASERCAN_TRIGGER_DISTANCE = 20;
    public static final double GEAR_RATIO = 5;
    public static final double INNER_WHEEL_RADIUS = 4.0;
  }

  public static final class WristConstants {
    public static final double GEAR_RATIO = 45;
    public static final double CLAW_LENGTH = Units.inchesToMeters(16.6);
    public static final double MIN_ANGLE = -0.93;
    public static final double MAX_ANGLE = 1.43;
    public static final double CUTOFF_ANGLE = 1.2;
    public static final double MOVE_ANGLE = 0.5;
  }

  public static final class ClimberConstants {
    public static final double GEAR_RATIO = 144;
    public static final double ARM_LENGTH = Units.inchesToMeters(16.6);
    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
    public static final double MIN_ANGLE = -Math.PI;
    public static final double MAX_ANGLE = Math.PI;
  }

  public static final class CANConstants {
    public static final int PIGEON_ID = 13;
    public static final int PDH_ID = 14;

    // FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right (relative
    // to pigeon
    // forward)
    // Order = DRIVE ID, TURN ID, CANCODER ID
    public static final int[] FL_IDS = {1, 5, 9};
    public static final int[] FR_IDS = {2, 6, 10};
    public static final int[] BL_IDS = {3, 7, 11};
    public static final int[] BR_IDS = {4, 8, 12};
    public static final int[][] MODULE_IDS = {FL_IDS, FR_IDS, BL_IDS, BR_IDS};

    public static final int ELEVATOR_ID = 15;
    public static final int ELEVATOR_FOLLOWER_ID = 16;
    public static final int WRIST_ID = 17;
    public static final int WRIST_ENCODER_ID = 18;
    public static final int CLAW_ID = 19;
    public static final int LASERCAN_ID = 20;
    public static final int CLIMBER_ID = 21;
    public static final int LEFT_CANDLE_ID = 22;
    public static final int RIGHT_CANDLE_ID = 23;
  }
}
