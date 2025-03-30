package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathfindingUtils;

public class BranchAlignFuture extends DriveToPose {

  boolean alignToRight;
  double futureTime = 0.1;

  public BranchAlignFuture(Drive drive, boolean alignToRight) {
    super(drive, new Pose2d(), drive::getReefPose);
    this.alignToRight = alignToRight;
  }

  @Override
  public void initialize() {
    Transform2d branch = alignToRight ? FieldConstants.RIGHT_BRANCH : FieldConstants.LEFT_BRANCH;

    setNewTarget(
        PathfindingUtils.getZoneReefPoseFuture(
            drive.getPose(), drive.getSpeeds(), futureTime, branch));
    super.initialize();
  }
}
