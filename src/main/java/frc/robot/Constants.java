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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static final Mode CURRENT_MODE = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "front_arducam";
    public static final String BACK_CAMERA_NAME = "back_arducam";
    public static final String LEFT_CAMERA_NAME = "left_arducam";
    public static final String RIGHT_CAMERA_NAME = "right_arducam";

    // Need to move cameras so they are 6.468 inches from the center of the robot is the y-axis
    // You can use the ideal transform below for sim, but it isn't a real transform
    public static final Transform3d FRONT_LEFT_IDEAL_TRANSFORM =
        new Transform3d(
            new Translation3d(
                DriveConstants.TRACK_WIDTH_X / 2,
                Units.inchesToMeters(6.468),
                Units.inchesToMeters(5)),
            new Rotation3d(0, 0.0, 0.0));

    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                DriveConstants.TRACK_WIDTH_X / 2, Units.inchesToMeters(3), Units.inchesToMeters(5)),
            new Rotation3d(0, 0.0, 0.0));

    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                DriveConstants.TRACK_WIDTH_X / 2,
                -Units.inchesToMeters(3),
                Units.inchesToMeters(5)),
            new Rotation3d(0, 0.0, 0.0));

    public static final Transform3d[] CAMERA_TRANSFORMS = {
      FRONT_LEFT_TRANSFORM, FRONT_RIGHT_TRANSFORM
    };

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.3;
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
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR =
        0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class PathfindingConstants {
    public static final PathConstraints CONSTRAINTS =
        new PathConstraints(4.0, 6.0, Units.degreesToRadians(720), Units.degreesToRadians(1080));

    public static final Pose2d[] BLUE_REEF_POSES =
        new Pose2d[] {
          new Pose2d(6, 4, Rotation2d.k180deg),
          new Pose2d(5.25, 2.75, new Rotation2d(Math.toRadians(120))),
          new Pose2d(3.75, 2.75, new Rotation2d(Math.toRadians(60))),
          new Pose2d(3, 4, Rotation2d.kZero),
          new Pose2d(3.75, 5.35, new Rotation2d(Math.toRadians(-60))),
          new Pose2d(5.25, 5.35, new Rotation2d(Math.toRadians(-120)))
        };

    public static final Pose2d BLUE_REEF_CENTER = new Pose2d(4.489323, 4.0259, new Rotation2d());

    public static final Pose2d[] RED_REEF_POSES =
        new Pose2d[] {
          new Pose2d(14.5, 4, Rotation2d.k180deg),
          new Pose2d(13.75, 2.75, new Rotation2d(Math.toRadians(120))),
          new Pose2d(12.3, 2.75, new Rotation2d(Math.toRadians(60))),
          new Pose2d(11.5, 4, Rotation2d.kZero),
          new Pose2d(12.3, 5.25, new Rotation2d(Math.toRadians(-60))),
          new Pose2d(13.75, 5.25, new Rotation2d(Math.toRadians(-120)))
        };

    public static final Pose2d RED_REEF_CENTER = new Pose2d(13.058902, 4.0259, new Rotation2d());

    public static final double X_LIMIT = 2.75;
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
  }

  public static final class DriveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;
    public static final double SLIP_CURRENT = 40;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final double[] ENCODER_OFFSETS = {2.888, -2.246 + Math.PI, -2.976, -2.745};
  }

  public static final class ElevatorConstants {
    public static final double GEAR_RATIO = 62.0 / 8.0;
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(25.0);
    public static final double PULLEY_RADIUS = Units.inchesToMeters(1.751) / 2;
    public static final double MIN_HEIGHT = Units.inchesToMeters(34.875);
    public static final double MAX_HEIGHT = MIN_HEIGHT + 1.75;
    public static final double MAGIC_NUMBER = Units.inchesToMeters(2.56718);
    public static final double CHARACTERIZATION_CUTOFF_HEIGHT = (MAX_HEIGHT - MIN_HEIGHT) / 1.5;
    public static final double[] POSITIONS = {0, 0.5, 0.9, 1.3, 1.7};
  }

  public static final class ClawConstants {
    public static final double LASERCAN_TRIGGER_DISTANCE = 1;
    public static final double GEAR_RATIO = 5;
    public static final double INNER_WHEEL_RADIUS = 4.0;
  }

  public static final class WristConstants {
    public static final double GEAR_RATIO = 12;
    public static final double CLAW_LENGTH = Units.inchesToMeters(16.6);
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 2 * Math.PI;
  }

  public static final class CANConstants {
    public static final int PIGEON_ID = 1;

    // FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right (relative
    // to pigeon
    // forward)
    // Order = DRIVE ID, TURN ID, CANCODER ID
    public static final int[] FL_IDS = {14, 12, 13};
    public static final int[] FR_IDS = {5, 3, 4};
    public static final int[] BL_IDS = {11, 9, 10};
    public static final int[] BR_IDS = {8, 6, 7};
    public static final int[][] MODULE_IDS = {FL_IDS, FR_IDS, BL_IDS, BR_IDS};

    public static final int ELEVATOR_ID = 15;
    public static final int ELEVATOR_FOLLOWER_ID = 16;
    public static final int WRIST_ID = 17;
    public static final int CLAW_ID = 18;
    public static final int LASERCAN_ID = 19;
  }
}
