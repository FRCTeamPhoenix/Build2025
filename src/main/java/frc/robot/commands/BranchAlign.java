package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathfindingUtils;

public class BranchAlign extends DriveToPose {

  boolean alignToRight;

  public BranchAlign(Drive drive, boolean alignToRight) {
    super(drive, new Pose2d());
    setNewConstraints(AutoConstants.FINE_LINEAR_CONSTRAINTS, AutoConstants.FINE_ANGLE_CONSTRAINTS);
    this.alignToRight = alignToRight;
  }

  @Override
  public void initialize() {
    Transform2d branch = alignToRight ? FieldConstants.RIGHT_BRANCH : FieldConstants.LEFT_BRANCH;

    setNewTarget(PathfindingUtils.getZoneReefPose(drive.getPose(), branch));
    super.initialize();
  }
}
