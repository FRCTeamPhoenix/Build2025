// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.cmd_candle_blink;
import frc.robot.commands.cmd_candle_on;
import frc.robot.subsystems.candle.Candle;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIO;
import frc.robot.subsystems.claw.ClawIOSim;
import frc.robot.subsystems.claw.ClawIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.photon.PhotonIO;
import frc.robot.subsystems.photon.PhotonIOReal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Photon photon;
  private final Claw claw;
  private final Elevator elevator;
  private final Candle candle = new Candle(6);

  //PID Controller
  private final PIDController steerPID = new PIDController(0.01, 0, 0.01);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Triggers
  private final Trigger xTrigger = controller.x();
  private final Trigger bTrigger = controller.b();
  private final Trigger aTrigger = controller.a();
  private final Trigger yTrigger = controller.y();

  //Subsystem sets
  private final Set<Subsystem> driveSet = new HashSet<Subsystem>();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
        photon = new Photon(
            drive::addVisionMeasurement,
            new PhotonIOReal(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM),
            new PhotonIOReal(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.FRONT_LEFT_TRANSFORM));
        elevator = new Elevator(new ElevatorIO() {});

        claw = new Claw(new ClawIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        photon = new Photon(

            drive::addVisionMeasurement,
            new PhotonIO() {
              // new PhotonIOSim(VisionConstants.FRONT_CAMERA_NAME,
              // VisionConstants.FRONT_LEFT_TRANSFORM, drive::getPose) {
            });
        claw = new Claw(new ClawIOSim());
        elevator = new Elevator(new ElevatorIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        photon = new Photon(
            drive::addVisionMeasurement,
            new PhotonIO() {
            });
        claw = new Claw(new ClawIO() {});
        elevator = new Elevator(new ElevatorIO() {});

        break;
    }


 SmartDashboard.putData("lights", new cmd_candle_on(candle));
 SmartDashboard.putData("blink_lights", new cmd_candle_blink(candle)); 


 candle.rainbowAnimate();

 // create a rainbow animation:
 // - max brightness
 // - half speed
 // - 64 LEDs

    // Configure PathPlanner commands
    configureNamedCommands();
    
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    steerPID.enableContinuousInput(-180, 180);
    driveSet.add(drive);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    xTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));
    bTrigger
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
                .ignoringDisable(true));

    yTrigger.whileTrue(Commands.defer(
        () -> AutoBuilder.pathfindToPose(determineZone(), PathfindingConstants.constraints, 0.0), driveSet));

    aTrigger.whileTrue(Commands.runOnce(claw::runForward, claw)).whileFalse(Commands.runOnce(claw::holdPosition, claw));

    elevator.setDefaultCommand(Commands.run(() -> {
      elevator.runSetpoint(
        MathUtil.clamp(elevator.getSetpoint() - controller.getLeftTriggerAxis() * 0.1 + controller.getRightTriggerAxis() * 0.1,
        0, ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT));
    }, elevator));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Raise Elevator Max", new MoveElevator(elevator, ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT));
    NamedCommands.registerCommand("Lower Elevator", new MoveElevator(elevator, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDrive() {
    return drive;
  }

  public Photon getPhoton() {
    return photon;
  }

  public CommandXboxController getController() {
    return controller;
  }

  public Pose2d determineZone() {
    boolean isRed = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d targetPose = drive.getPose();
    Pose2d[] reefPoses = isRed ? PathfindingConstants.redReefPoses
        : PathfindingConstants.blueReefPoses;

    Pose2d reefCenter = isRed ? PathfindingConstants.redReefCenter
        : PathfindingConstants.blueReefCenter;

    Transform2d translatedRobot = targetPose.minus(reefCenter);

    double theta = Units.radiansToDegrees(Math.atan2(translatedRobot.getY(), translatedRobot.getX()));

    if (Math.abs(translatedRobot.getX()) < PathfindingConstants.xLimit
        && Math.abs(translatedRobot.getY()) < PathfindingConstants.yLimit) {
      if (-30 < theta && theta < 30) {
        targetPose = reefPoses[0];
      } else if (-30 > theta && theta > -90) {
        targetPose = reefPoses[1];
      } else if (-90 > theta && theta > -150) {
        targetPose = reefPoses[2];
      } else if ((-150 > theta && theta > -180) || (150 < theta && theta < 180)) {
        targetPose = reefPoses[3];
      } else if (90 < theta && theta < 150) {
        targetPose = reefPoses[4];
      } else if (30 < theta && theta < 90) {
        targetPose = reefPoses[5];
      }
    }

    return targetPose;
  }

  public Pose2d[] generateZone() {
    boolean isRed = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d reefCenter = isRed ? PathfindingConstants.redReefCenter
        : PathfindingConstants.blueReefCenter;

    List<Pose2d> zoneTrajectory = new ArrayList<Pose2d>();

    for (Transform2d transform : PathfindingConstants.zoneTransforms) {
      zoneTrajectory.add(reefCenter.plus(transform));
    }
    return zoneTrajectory.toArray(new Pose2d[0]);
  }
}
