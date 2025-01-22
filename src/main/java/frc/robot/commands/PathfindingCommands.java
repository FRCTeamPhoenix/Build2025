package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathfindingConstants;

public class PathfindingCommands {
    public static Command pathToPlayerStation(int id) {
        boolean isRed = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
                
        Pose2d[] stations = PathfindingConstants.bluePlayerStationPoses;
        if (isRed) {stations = PathfindingConstants.bluePlayerStationPoses;}

        //AutoBuilder.getCurrentPose();
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                stations[id - 1],
                PathfindingConstants.constraints,
                0.0 // Goal end velocity in meters/sec
        );       
    }
}
