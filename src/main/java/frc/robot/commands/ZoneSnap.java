package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathfindingUtils;

public class ZoneSnap extends DriveToPose {

  public ZoneSnap(Drive drive) {
    super(drive, new Pose2d());
  }

  @Override
  public void initialize() {
    setNewTarget(
        PathfindingUtils.getZoneReefPose(
            drive.getPose(), PathfindingConstants.REEF_BUFFER_TRANSFORM));
    super.initialize();
  }
}
