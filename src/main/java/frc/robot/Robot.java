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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AutoComposer;
import frc.robot.util.PathfindingUtils;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final Field2d field = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        System.out.println("Starting AdvantageKit on a real robot");
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    PathfindingUtils.warmupCommand().schedule();
    Logger.recordOutput("PoseAlignment/AtGoal", false);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    SmartDashboard.putBoolean("Zero LimeLight Gyro", false);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    field.setRobotPose(robotContainer.getDrive().getPose());
    SmartDashboard.putData("Field", field);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (SmartDashboard.getBoolean("Use Auto Composer", false)) {
      autonomousCommand =
          AutoComposer.composeAuto(
              SmartDashboard.getString("Composer Input", "1a4"),
              robotContainer::getElevatorCommands,
              robotContainer::getScoringCommand,
              robotContainer::getIntakingCommand,
              robotContainer.getDrive());
    }

    robotContainer.getSuperstructure().setState(0);

    if (!robotContainer.getDrive().getOffsetDone()) {
      robotContainer
          .getDrive()
          .setMegatagOffset(
              robotContainer
                  .getDrive()
                  .getReefPose()
                  .getRotation()
                  .minus(robotContainer.getDrive().getMegatagRotation()));
    }

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.getDrive().runVelocity(new ChassisSpeeds());
    robotContainer
        .getSuperstructure()
        .setElevatorManualGoal(robotContainer.getSuperstructure().getElevatorHeight());
    robotContainer
        .getSuperstructure()
        .setWristManualGoal(robotContainer.getSuperstructure().getWristAngle());
    robotContainer.getClaw().stop();

    if (!robotContainer.getDrive().getOffsetDone()) {
      robotContainer
          .getDrive()
          .setMegatagOffset(
              robotContainer
                  .getDrive()
                  .getReefPose()
                  .getRotation()
                  .minus(robotContainer.getDrive().getMegatagRotation()));
    }

    Logger.recordOutput("ZoneSnapping/ZoneMap", PathfindingUtils.generateZone());
    field.getObject("ZoneMap").setPoses(PathfindingUtils.generateZone());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (Constants.CURRENT_MODE == frc.robot.Constants.Mode.SIM) {
      SimulatedArena.getInstance().simulationPeriodic();
      Pose3d[] corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
      Logger.recordOutput("Simulation/CoralPositions", corals);
      Logger.recordOutput("Simulation/Pose", robotContainer.swerveSim.getSimulatedDriveTrainPose());
      SwerveModuleState[] arr = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        arr[i] = robotContainer.swerveSim.getModules()[i].getCurrentState();
      }
      Logger.recordOutput("Simulation/States", arr);
    }
  }
}
