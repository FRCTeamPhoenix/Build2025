package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathfindingUtils;

public class ZoneSnap extends DriveToPose {

  public ZoneSnap(Drive drive) {
    super(drive, new Pose2d());
  }

  @Override
  public void initialize() {
    setNewTarget(PathfindingUtils.getZone(drive.getPose()), Rotation2d.k180deg);
    super.initialize();
  }
}
