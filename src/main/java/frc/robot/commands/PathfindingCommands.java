package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathfindingConstants;

public class PathfindingCommands {
    public static Command pathToPlayerStation(int id) {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        Pose2d[] stations = PathfindingConstants.bluePlayerStationPoses;
        if (isRed) {
            stations = PathfindingConstants.bluePlayerStationPoses;
        }

        // AutoBuilder.getCurrentPose();
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                stations[id - 1],
                PathfindingConstants.constraints,
                0.0 // Goal end velocity in meters/sec
        );
    }

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
                (speeds, feedforwards) -> {
                },
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
}
