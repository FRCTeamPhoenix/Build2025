package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class SlowToPose extends DriveToPose {

  boolean alignToRight;

  public SlowToPose(Drive drive, Pose2d pose) {
    super(drive, pose, drive::getPose);
  }
}
