package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.BranchAlign;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoComposer {

  public static Command composeAuto(
      String routine,
      Supplier<Command[]> elevatorCommands,
      Supplier<Command> scoringCommand,
      Supplier<Command> intakeCommand,
      Supplier<Command> stopIntakeCommand,
      Supplier<Command> dropIntakeCommand,
      Supplier<Command> backupCommand,
      Supplier<Command> getStationAlignLeft,
      Supplier<Command> getStationAlignRight,
      BooleanSupplier intakeSensor,
      Drive drive,
      Boolean isRed) {
    Command returnCommand = Commands.none();
    String allianceName = isRed ? ".red" : ".blue";
    returnCommand.setName(routine + allianceName);
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
                  corrected[i], elevatorCommands, scoringCommand, stopIntakeCommand, drive, isRed));
        }
        if (corrected[i].charAt(0) == 'r' || corrected[i].charAt(0) == 'l') {
          commandArray.add(
              intakingRoutine(
                  corrected[i],
                  intakeCommand,
                  dropIntakeCommand,
                  backupCommand,
                  getStationAlignLeft,
                  getStationAlignRight,
                  intakeSensor,
                  drive,
                  corrected[i - 1].substring(0, 1)));
        }
        if (corrected[i].charAt(0) == 'b') {
          commandArray.add(backupCommand.get());
          corrected[i] = corrected[i - 1];
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
      Supplier<Command[]> scoringCommands,
      Supplier<Command> shootCommand,
      Supplier<Command> stopIntakeCommand,
      Drive drive,
      Boolean isRed) {
    Command returnCommand;

    char[] routineSplit = routine.toLowerCase().toCharArray();

    Pose3d[] reefPoses = isRed ? AutoConstants.RED_REEF_POSES : AutoConstants.BLUE_REEF_POSES;

    try {
      int reefFace = routineSplit[0] - '0';
      int level = routineSplit[2] - '0';

      if (routineSplit[1] == 'a') {
        returnCommand =
            AutoBuilder.pathfindToPose(
                    reefPoses[reefFace - 1].toPose2d().plus(FieldConstants.REEF_LEFT_PATH_BUFFER),
                    AutoConstants.CONSTRAINTS,
                    MetersPerSecond.of(3))
                .alongWith(stopIntakeCommand.get());

        returnCommand =
            returnCommand.andThen(
                scoringCommands.get()[level - 1].alongWith(
                    new DeferredCommand(() -> new BranchAlign(drive, false), Set.of())
                        .withTimeout(1.2)));
      } else {
        returnCommand =
            AutoBuilder.pathfindToPose(
                    reefPoses[reefFace - 1].toPose2d().plus(FieldConstants.REEF_RIGHT_PATH_BUFFER),
                    AutoConstants.CONSTRAINTS,
                    MetersPerSecond.of(3))
                .alongWith(stopIntakeCommand.get());

        returnCommand =
            returnCommand.andThen(
                scoringCommands.get()[level - 1].alongWith(
                    new DeferredCommand(() -> new BranchAlign(drive, true), Set.of())
                        .withTimeout(1.2)));
      }
      returnCommand = returnCommand.andThen(shootCommand.get());
    } catch (Exception e) {
      System.out.println(e);
      System.out.println("Failed to generate scoring command");
      returnCommand = Commands.none();
    }

    return returnCommand;
  }

  public static Command intakingRoutine(
      String routine,
      Supplier<Command> intakeCommand,
      Supplier<Command> dropIntakeCommand,
      Supplier<Command> dropElevatorCommand,
      Supplier<Command> alignStationLeft,
      Supplier<Command> alignStationRight,
      BooleanSupplier intakeSensor,
      Drive drive,
      String lastPosition) {

    Command returnCommand;

    char[] routineSplit = routine.toLowerCase().toCharArray();

    try {
      if (routineSplit[0] == 'r') {
        returnCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastPosition + "r"));
        returnCommand = returnCommand.andThen(alignStationRight.get());
      } else {
        returnCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastPosition + "l"));
        returnCommand = returnCommand.andThen(alignStationLeft.get());
      }
      returnCommand = returnCommand.alongWith(intakeCommand.get());
      returnCommand = returnCommand.until(intakeSensor);
      returnCommand = returnCommand.andThen(dropIntakeCommand.get());
    } catch (Exception e) {
      System.out.println(e);
      System.out.println("Failed to generate intake command");
      returnCommand = Commands.none();
    }

    return returnCommand;
  }
}
