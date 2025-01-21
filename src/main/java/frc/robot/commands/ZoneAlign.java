package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.PathfindingConstants;
import frc.robot.util.PhoenixUtils;

public class ZoneAlign extends Command {

    private Command pathfindingCommand;

    @Override
    public void initialize() {
        boolean isRed = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
        
        Pose2d[] zones = PathfindingConstants.blueReefPoses;
        Pose2d targetPose = PathfindingConstants.blueReefPoses[0];

        if (isRed) {zones = PathfindingConstants.redReefPoses; targetPose = PathfindingConstants.redReefPoses[0];}

        int i;
        for (i = 0; i < zones.length; i++){
            if (PhoenixUtils.getDistance(AutoBuilder.getCurrentPose(), zones[i]) < PhoenixUtils.getDistance(AutoBuilder.getCurrentPose(), targetPose)) {
                targetPose = zones[i];
            }
        }

        
        Logger.recordOutput(
          "ZoneAlign/TargetPose",
          targetPose);

        Logger.recordOutput("ZoneAlign/Active", true);

        //AutoBuilder.getCurrentPose();
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                PathfindingConstants.constraints,
                0.0 // Goal end velocity in meters/sec
        );
        
        pathfindingCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("ZoneAlign/Active", false);
        pathfindingCommand.end(false);
        pathfindingCommand.cancel();
    }
}