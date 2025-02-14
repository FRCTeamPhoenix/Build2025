package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PathfindingUtils;

public class DriveToPlayerStation extends DriveToPose {

  public DriveToPlayerStation(Drive drive) {
    super(drive, new Pose2d());
  }

  @Override
  public void initialize() {
    setNewTarget(
        PathfindingUtils.getClosestPlayerStation(
            drive.getPose(), PathfindingConstants.CENTER_PLAYER_STATION));
    super.initialize();
  }
}
