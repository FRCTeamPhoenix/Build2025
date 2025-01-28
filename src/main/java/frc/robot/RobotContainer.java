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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.photon.PhotonIO;
import frc.robot.subsystems.photon.PhotonIOReal;
import frc.robot.subsystems.photon.PhotonIOSim;
import frc.robot.util.PhoenixUtils;

import java.util.HashSet;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.Logger;


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
  private final PIDController steerPID = new PIDController(0.01, 0, 0.01);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Triggers
  private final Trigger xTrigger = controller.x();
  private final Trigger bTrigger = controller.b();
  private final Trigger aTrigger = controller.a();
  private final Trigger yTrigger = controller.y();

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
            new PhotonIOReal(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM));
           // new PhotonIOReal(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM));
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
            new PhotonIOSim(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.FRONT_LEFT_IDEAL_TRANSFORM, drive::getPose));
            //new PhotonIOSim(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM, drive::getPose));
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
        break;
    }

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

    Set<Subsystem> sysSet = new HashSet<>();
    sysSet.add(drive);

    yTrigger.whileTrue(Commands.defer(
        () -> AutoBuilder.pathfindToPose(determineZone(), PathfindingConstants.constraints, 0.0), sysSet));

    aTrigger
        .whileTrue(new AlignToTag(0, photon, drive));
  }

  /**
   * 
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

    Pose2d[] zones = PathfindingConstants.blueReefPoses;
    Pose2d targetPose = PathfindingConstants.blueReefPoses[0];

    if (isRed) {
      zones = PathfindingConstants.redReefPoses;
      targetPose = PathfindingConstants.redReefPoses[0];
    }

    for (int i = 0; i < zones.length; i++) {
      if (PhoenixUtils.getDistance(drive.getPose(), zones[i]) < PhoenixUtils.getDistance(drive.getPose(), targetPose)) {
        targetPose = zones[i];
      }
    }

    return targetPose;
  }
}
