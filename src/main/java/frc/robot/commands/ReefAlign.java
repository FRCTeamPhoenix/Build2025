package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PhoenixUtils;

public class ReefAlign extends DriveToPose {

  boolean alignToRight;

  public ReefAlign(Drive drive, boolean alignToRight) {
    super(drive, new Pose2d());
    setNewConstraints(
        PathfindingConstants.FINE_LINEAR_CONSTRAINTS, PathfindingConstants.FINE_ANGLE_CONSTRAINTS);
    this.alignToRight = alignToRight;
  }

  @Override
  public void initialize() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose3d drivePose = new Pose3d(drive.getPose());
    Pose3d targetPose = new Pose3d(1000, 1000, 1000, Rotation3d.kZero);
    Pose3d[] tagPoses =
        isRed ? PathfindingConstants.RED_REEF_TAG_POSES : PathfindingConstants.BLUE_REEF_TAG_POSES;
    for (Pose3d tag : tagPoses) {
      double targetDistance = PhoenixUtils.getDistance(drivePose, targetPose);
      double tagDistance = PhoenixUtils.getDistance(drivePose, tag);
      if (tagDistance < targetDistance) {
        targetPose = tag;
      }
    }

    Transform3d branch =
        alignToRight ? PathfindingConstants.RIGHT_BRANCH : PathfindingConstants.LEFT_BRANCH;
    targetPose = targetPose.plus(branch);

    setNewTarget(targetPose.toPose2d(), Rotation2d.k180deg);
    super.initialize();
  }
}
