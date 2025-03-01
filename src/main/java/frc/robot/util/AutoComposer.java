package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.BranchAlign;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.SlowToPose;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class AutoComposer {

  public static Command composeAuto(
      String routine,
      Supplier<Command[]> elevatorCommands,
      Supplier<Command> scoringCommand,
      Supplier<Command> intakeCommand,
      Drive drive) {
    Command returnCommand = Commands.none();
    List<Command> commandArray = new ArrayList<Command>();

    try {
      String[] individuals = routine.split("\\.");
      String[] corrected = new String[individuals.length + 1];
      for (int i = 1; i < corrected.length; i++) {
        corrected[i] = individuals[i - 1];
      }
      corrected[0] = "000";
      for (int i = 0; i < corrected.length; i++) {
        if (Character.isDigit(corrected[i].charAt(0)) && corrected[i].charAt(0) != '0') {
          commandArray.add(
              scoringRoutine(
                  corrected[i],
                  elevatorCommands,
                  scoringCommand,
                  drive,
                  corrected[i - 1].substring(0, 1)));
        }
        if (corrected[i].charAt(0) == 'r' || corrected[i].charAt(0) == 'l') {
          commandArray.add(
              intakingRoutine(
                  corrected[i], intakeCommand, drive, corrected[i - 1].substring(0, 1)));
        }
      }
    } catch (Exception e) {
      System.out.println(e);
      System.out.println("Failed to generate auto routine");
      commandArray.add(Commands.none());
    }

    for (Command next : commandArray) {
      returnCommand = returnCommand.andThen(next);
    }

    return returnCommand;
  }

  public static Command scoringRoutine(
      String routine,
      Supplier<Command[]> elevatorCommands,
      Supplier<Command> scoringCommand,
      Drive drive,
      String lastPosition) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    Command returnCommand;

    char[] routineSplit = routine.toLowerCase().toCharArray();

    Pose3d[] reefPoses = isRed ? AutoConstants.RED_REEF_POSES : AutoConstants.BLUE_REEF_POSES;

    try {
      int reefFace = routineSplit[0] - '0';
      int level = routineSplit[2] - '0';

      if (lastPosition != "0") {
        returnCommand =
            AutoBuilder.pathfindToPose(
                reefPoses[reefFace - 1].toPose2d().plus(AutoConstants.PATHING_BUFFER),
                AutoConstants.CONSTRAINTS);

      } else {
        returnCommand =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(lastPosition + Integer.toString(reefFace)),
                AutoConstants.CONSTRAINTS);
      }
      if (routineSplit[1] == 'a') {
        returnCommand =
            returnCommand.andThen(
                elevatorCommands.get()[level - 1].alongWith(
                    new DeferredCommand(() -> new BranchAlign(drive, false), Set.of())));
      } else {
        returnCommand =
            returnCommand.andThen(
                elevatorCommands.get()[level - 1].alongWith(
                    new DeferredCommand(() -> new BranchAlign(drive, true), Set.of())));
      }
      returnCommand = returnCommand.andThen(scoringCommand.get());
      returnCommand =
          returnCommand.andThen(
              new SlowToPose(
                      drive, reefPoses[reefFace - 1].toPose2d().plus(AutoConstants.PATHING_BUFFER))
                  .alongWith(elevatorCommands.get()[4]));
    } catch (Exception e) {
      System.out.println(lastPosition);
      System.out.println(lastPosition.getClass());
      System.out.println(e);
      System.out.println("Failed to generate scoring command");
      returnCommand = Commands.none();
    }

    return returnCommand;
  }

  public static Command intakingRoutine(
      String routine, Supplier<Command> intakeCommand, Drive drive, String lastPosition) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    Command returnCommand;

    char[] routineSplit = routine.toLowerCase().toCharArray();

    Pose2d[] playerStations =
        isRed
            ? AutoConstants.PATHING_RED_PLAYER_STATIONS
            : AutoConstants.PATHING_BLUE_PLAYER_STATIONS;

    try {
      if (routineSplit[0] == 'r') {
        returnCommand =
            AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile(lastPosition + "r"), AutoConstants.CONSTRAINTS)
                .andThen(
                    new DriveToPose(
                        drive,
                        playerStations[1].plus(FieldConstants.CENTER_PLAYER_STATION),
                        drive::getPose));
      } else {
        returnCommand =
            AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile(lastPosition + "l"), AutoConstants.CONSTRAINTS)
                .andThen(
                    new DriveToPose(
                        drive,
                        playerStations[0].plus(FieldConstants.CENTER_PLAYER_STATION),
                        drive::getPose));
      }
      returnCommand = returnCommand.alongWith(intakeCommand.get());
    } catch (Exception e) {
      System.out.println(e);
      System.out.println("Failed to generate intake command");
      returnCommand = Commands.none();
    }

    return returnCommand;
  }
}
