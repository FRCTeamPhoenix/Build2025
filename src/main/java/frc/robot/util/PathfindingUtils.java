package frc.robot.util;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathfindingConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PathfindingUtils {
  /**
   * Create a command to warmup the pathfinder and pathfinding command
   *
   * @return Pathfinding warmup command
   */
  public static Command warmupCommand() {
    Logger.recordOutput("Pathfinding Ready", false);
    return new PathfindingCommand(
            new Pose2d(1.6, 4.0, Rotation2d.kZero),
            new PathConstraints(4, 3, 4, 4),
            () -> new Pose2d(1.5, 4, Rotation2d.kZero),
            ChassisSpeeds::new,
            (speeds, feedforwards) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 15.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55))
        .andThen(Commands.runOnce(() -> Logger.recordOutput("Pathfinding Ready", true)))
        .ignoringDisable(true);
  }

  public static Pose2d[] generateZone() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d reefCenter =
        isRed ? PathfindingConstants.RED_REEF_CENTER : PathfindingConstants.BLUE_REEF_CENTER;

    List<Pose2d> zoneTrajectory = new ArrayList<Pose2d>();

    for (Transform2d transform : PathfindingConstants.ZONE_TRANSFORMS) {
      zoneTrajectory.add(reefCenter.plus(transform));
    }
    return zoneTrajectory.toArray(new Pose2d[0]);
  }

  public static Pose2d getZoneReefPose(Pose2d odometryPose, Transform2d buffer) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d targetPose = odometryPose;
    Pose3d[] reefPoses =
        isRed ? PathfindingConstants.RED_REEF_TAG_POSES : PathfindingConstants.BLUE_REEF_TAG_POSES;

    Pose2d reefCenter =
        isRed ? PathfindingConstants.RED_REEF_CENTER : PathfindingConstants.BLUE_REEF_CENTER;

    Transform2d translatedRobot = targetPose.minus(reefCenter);

    double theta =
        Units.radiansToDegrees(Math.atan2(translatedRobot.getY(), translatedRobot.getX()));

    if (Math.abs(translatedRobot.getX()) < PathfindingConstants.X_LIMIT
        && Math.abs(translatedRobot.getY()) < PathfindingConstants.Y_LIMIT) {
      if (-30 < theta && theta < 30) {
        targetPose = reefPoses[0].toPose2d().plus(buffer);
      } else if (-30 > theta && theta > -90) {
        targetPose = reefPoses[1].toPose2d().plus(buffer);
      } else if (-90 > theta && theta > -150) {
        targetPose = reefPoses[2].toPose2d().plus(buffer);
      } else if ((-150 > theta && theta > -180) || (150 < theta && theta < 180)) {
        targetPose = reefPoses[3].toPose2d().plus(buffer);
      } else if (90 < theta && theta < 150) {
        targetPose = reefPoses[4].toPose2d().plus(buffer);
      } else if (30 < theta && theta < 90) {
        targetPose = reefPoses[5].toPose2d().plus(buffer);
      }
    }
    return targetPose;
  }

  public static Pose2d getClosestPlayerStation(Pose2d odometryPose, Transform2d buffer) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d[] stations =
        isRed
            ? PathfindingConstants.RED_PLAYER_STATIONS
            : PathfindingConstants.BLUE_PLAYER_STATIONS;

    Logger.recordOutput("Players", stations);
    Pose2d target = odometryPose.nearest(Arrays.asList(stations));

    return target.plus(buffer);
  }
}
