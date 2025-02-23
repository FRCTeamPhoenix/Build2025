package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class AutoComposer {

  public static Command composeAuto(
      String routine,
      Supplier<Command[]> scoringCommand,
      Supplier<Command> intakeCommand,
      Drive drive) {
    Command returnCommand = new WaitCommand(0);
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose3d[] reefPoses =
        isRed ? PathfindingConstants.RED_REEF_TAG_POSES : PathfindingConstants.BLUE_REEF_TAG_POSES;
    Pose2d[] playerStations =
        isRed
            ? PathfindingConstants.RED_PLAYER_STATIONS
            : PathfindingConstants.BLUE_PLAYER_STATIONS;

    String[] individuals = routine.split("\\.");
    List<Command> commands = new ArrayList<Command>();
    for (String individual : individuals) {
      if (Character.isDigit(individual.charAt(0))) {
        if (individual.charAt(0) - '0' <= reefPoses.length - 1) {
          if (individual.toLowerCase().charAt(1) == 'a') {
            int level = individual.toLowerCase().charAt(2) - '0';
            commands.add(
                AutoBuilder.pathfindToPose(
                        reefPoses[individual.charAt(0) - '0']
                            .toPose2d()
                            .plus(PathfindingConstants.CENTER_PLAYER_STATION),
                        PathfindingConstants.CONSTRAINTS)
                    .andThen(
                        new DriveToPose(
                            drive,
                            reefPoses[individual.charAt(0) - '0']
                                .toPose2d()
                                .plus(PathfindingConstants.LEFT_BRANCH)))
                    .andThen(scoringCommand.get()[level - 1]));
          } else {
            int level = individual.toLowerCase().charAt(2) - '0';
            commands.add(
                AutoBuilder.pathfindToPose(
                        reefPoses[individual.charAt(0) - '0']
                            .toPose2d()
                            .plus(PathfindingConstants.CENTER_PLAYER_STATION),
                        PathfindingConstants.CONSTRAINTS)
                    .andThen(
                        new DriveToPose(
                            drive,
                            reefPoses[individual.charAt(0) - '0']
                                .toPose2d()
                                .plus(PathfindingConstants.RIGHT_BRANCH)))
                    .andThen(scoringCommand.get()[level - 1]));
          }
        }

        if (individual.charAt(0) == 'i') {
          if (individual.toLowerCase().charAt(1) == 'l') {
            commands.add(
                AutoBuilder.pathfindToPose(
                        playerStations[individual.charAt(0) - '0'].plus(
                            PathfindingConstants.REEF_BUFFER_TRANSFORM),
                        PathfindingConstants.CONSTRAINTS)
                    .andThen(
                        new DriveToPose(
                            drive,
                            reefPoses[individual.charAt(0) - '0']
                                .toPose2d()
                                .plus(PathfindingConstants.LEFT_BRANCH))));
            // .andThen(scoringCommand.get()));
          } else {
            commands.add(
                AutoBuilder.pathfindToPose(
                        reefPoses[individual.charAt(0) - '0']
                            .toPose2d()
                            .plus(PathfindingConstants.REEF_BUFFER_TRANSFORM),
                        PathfindingConstants.CONSTRAINTS)
                    .andThen(
                        new DriveToPose(
                            drive,
                            reefPoses[individual.charAt(0) - '0']
                                .toPose2d()
                                .plus(PathfindingConstants.RIGHT_BRANCH))));
            // .andThen(scoringCommand.get()));
          }
        }
      }
    }

    for (Command next : commands) {
      returnCommand = returnCommand.andThen(next);
    }

    return returnCommand;
  }
}
