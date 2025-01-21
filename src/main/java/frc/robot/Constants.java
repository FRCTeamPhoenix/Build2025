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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
    public static final String FRONT_CAMERA_NAME = "front_arducam";
    public static final String BACK_CAMERA_NAME = "back_arducam";
    public static final String LEFT_CAMERA_NAME = "left_arducam";
    public static final String RIGHT_CAMERA_NAME = "right_arducam";

    public static final Transform3d test_Transform = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d FRONT_TRANSFORM = new Transform3d(
        new Translation3d(0.3048, 0.0, 0.12065), new Rotation3d(0, Math.toRadians(20), 0.0));
    public static final Transform3d BACK_TRANSFORM = new Transform3d(
        new Translation3d(-0.3048, 0.0, 0.12065),
        new Rotation3d(0, Math.toRadians(20), Math.PI));
    public static final Transform3d LEFT_TRANSFORM = new Transform3d(
        new Translation3d(0.0, -0.3048, 0.12065),
        new Rotation3d(0, Math.toRadians(20), (Math.PI / 2.0)));
    public static final Transform3d RIGHT_TRANSFORM = new Transform3d(
        new Translation3d(0.0, 0.3048, 0.12065),
        new Rotation3d(0, Math.toRadians(20), (-Math.PI / 2.0)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.3;
    public static double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAMERA_STD_DEV_FACTORS = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class DriveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;
    public static final double SLIP_CURRENT = 40;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final double[] ENCODER_OFFSETS = {
      2.888, -2.246 + Math.PI, -2.976, -2.745
    };
  }

  public static final class ClawConstants {
    public static final int CLAW_ID = 15;

    public static final double GEAR_RATIO = 5;
  }

  public static final class CANConstants {
    public static final int PIGEON_ID = 1;

    // FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right (relative
    // to pigeon
    // forward)
    // Order = DRIVE ID, TURN ID, CANCODER ID
    public static final int[] FL_IDS = { 14, 12, 13 };
    public static final int[] FR_IDS = { 5, 3, 4 };
    public static final int[] BL_IDS = { 11, 9, 10 };
    public static final int[] BR_IDS = { 8, 6, 7 };

    public static final int[][] MODULE_IDS = { FL_IDS, FR_IDS, BL_IDS, BR_IDS };
  }
}
