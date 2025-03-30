package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathfindingUtils;

public class ZoneSnap extends DriveToPose {

  double futureTime = 0.1;

  public ZoneSnap(Drive drive) {
    super(drive, new Pose2d(), drive::getPose);
  }

  @Override
  public void initialize() {
    setNewTarget(
        PathfindingUtils.getZoneReefPoseFuture(
            drive.getPose(), drive.getSpeeds(), futureTime, FieldConstants.REEF_BUFFER_TRANSFORM));

    super.initialize();
  }
}
