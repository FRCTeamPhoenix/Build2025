package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveToPlayerStation extends DriveToPose {

  private boolean alignToRight;

  public DriveToPlayerStation(Drive drive, boolean alignToRight) {
    super(drive, new Pose2d(), drive::getPose);
    this.alignToRight = alignToRight;
  }

  @Override
  public void initialize() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    Pose2d[] stations =
        isRed ? FieldConstants.RED_PLAYER_STATIONS : FieldConstants.BLUE_PLAYER_STATIONS;

    int side = alignToRight ? 1 : 0;
    setNewTarget(stations[side].plus(FieldConstants.CENTER_PLAYER_STATION));
    super.initialize();
  }
}
