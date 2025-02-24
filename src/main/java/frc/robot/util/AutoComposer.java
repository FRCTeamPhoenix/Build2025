package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.BranchAlign;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class AutoComposer {

  public static Command composeAuto(
      String routine,
      Supplier<Command[]> scoringCommand,
      Supplier<Command> intakeCommand,
      Drive drive) {
    Command returnCommand = Commands.none();
    List<Command> commandArray = new ArrayList<Command>();

    try {
      String[] individuals = routine.split("\\.");
      for (String individual : individuals) {
        if (Character.isDigit(individual.charAt(0))) {
          commandArray.add(scoringRoutine(individual, scoringCommand, drive));
        }
        if (individual.charAt(0) == 'i') {
          commandArray.add(intakingRoutine(individual, intakeCommand));
        }
      }
    } catch (Exception e) {
      commandArray.add(Commands.none());
      System.out.println("Failed to generate auto");
    }

    for (Command next : commandArray) {
      returnCommand = returnCommand.andThen(next);
    }

    return returnCommand;
  }

  public static Command scoringRoutine(
      String routine, Supplier<Command[]> scoringCommands, Drive drive) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    Command returnCommand;

    char[] routineSplit = routine.toLowerCase().toCharArray();

    Pose3d[] reefPoses =
        isRed ? PathfindingConstants.RED_REEF_TAG_POSES : PathfindingConstants.BLUE_REEF_TAG_POSES;

    try {
      int reefFace = routineSplit[0] - '0';
      if (routineSplit[1] == 'a') {
        returnCommand =
            AutoBuilder.pathfindToPose(
                    reefPoses[reefFace].toPose2d().plus(PathfindingConstants.REEF_BUFFER_TRANSFORM),
                    PathfindingConstants.CONSTRAINTS)
                .andThen(new DeferredCommand(() -> new BranchAlign(drive, false), Set.of()));
      } else {
        returnCommand =
            AutoBuilder.pathfindToPose(
                    reefPoses[reefFace].toPose2d().plus(PathfindingConstants.REEF_BUFFER_TRANSFORM),
                    PathfindingConstants.CONSTRAINTS)
                .andThen(new DeferredCommand(() -> new BranchAlign(drive, true), Set.of()));
      }
      int level = routineSplit[2] - '0';
      returnCommand = returnCommand.andThen(scoringCommands.get()[level - 1]);
    } catch (Exception e) {
      System.out.println("Failed to generate scoring command");
      returnCommand = Commands.none();
    }

    return returnCommand;
  }

  public static Command intakingRoutine(String routine, Supplier<Command> intakeCommand) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    Command returnCommand;

    char[] routineSplit = routine.toLowerCase().toCharArray();

    Pose2d[] playerStations =
        isRed
            ? PathfindingConstants.RED_PLAYER_STATIONS
            : PathfindingConstants.BLUE_PLAYER_STATIONS;

    try {
      if (routineSplit[1] == 'r') {
        returnCommand =
            AutoBuilder.pathfindToPose(
                playerStations[0].plus(PathfindingConstants.CENTER_PLAYER_STATION),
                PathfindingConstants.CONSTRAINTS);
      } else {
        returnCommand =
            AutoBuilder.pathfindToPose(
                playerStations[1].plus(PathfindingConstants.CENTER_PLAYER_STATION),
                PathfindingConstants.CONSTRAINTS);
      }
      returnCommand = returnCommand.andThen(intakeCommand.get());
    } catch (Exception e) {
      System.out.println("Failed to generate intake command");
      returnCommand = Commands.none();
    }

    return returnCommand;
  }
}
