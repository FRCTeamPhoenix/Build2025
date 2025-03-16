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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoElevator;
import frc.robot.commands.BranchAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPlayerStation;
import frc.robot.commands.ZoneSnap;
import frc.robot.subsystems.candle.CANdleIO;
import frc.robot.subsystems.candle.CANdleIOReal;
import frc.robot.subsystems.candle.CANdleSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMapleSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.subsystems.superstructure.claw.ClawIOSim;
import frc.robot.subsystems.superstructure.claw.ClawIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.superstructure.wrist.WristIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.visualizer.Visualizer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Claw claw;
  private final Elevator elevator;
  private final Visualizer visualizer;
  private final Superstructure superstructure;
  private final Wrist wrist;
  private final Climber climber;
  private final CANdleSubsystem candle;

  private final DriveTrainSimulationConfig driveSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  2))
          .withTrackLengthTrackWidth(
              Meters.of(DriveConstants.TRACK_WIDTH_X), Meters.of(DriveConstants.TRACK_WIDTH_Y))
          .withBumperSize(Inches.of(30), Inches.of(30));

  public SwerveDriveSimulation swerveSim;

  public int selectedScore = 5;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Triggers
  private final Trigger driverBTrigger = driverController.b();
  private final Trigger driverXTrigger = driverController.x();
  private final Trigger driverLBTrigger = driverController.leftBumper();
  private final Trigger driverRBTrigger = driverController.rightBumper();
  private final Trigger driverLTTrigger = driverController.leftTrigger();
  private final Trigger driverRTTrigger = driverController.rightTrigger();
  private final Trigger driverLeftPadTrigger = driverController.povLeft();
  private final Trigger driverDownPadTrigger = driverController.povDown();

  private final Trigger operatorATrigger = operatorController.a();
  private final Trigger operatorBTrigger = operatorController.b();
  private final Trigger operatorXTrigger = operatorController.x();
  private final Trigger operatorYTrigger = operatorController.y();
  private final Trigger operatorLBTrigger = operatorController.leftBumper();
  private final Trigger operatorRBTrigger = operatorController.rightBumper();
  private final Trigger operatorLTTrigger = operatorController.leftTrigger();
  private final Trigger operatorRTTrigger = operatorController.rightTrigger();
  private final Trigger operatorLSTrigger = operatorController.leftStick();
  private final Trigger operatorRSTrigger = operatorController.rightStick();
  private final Trigger operatorUpPadTrigger = operatorController.povUp();
  private final Trigger operatorDownPadTrigger = operatorController.povDown();
  private final Trigger operatorLeftPadTrigger = operatorController.povLeft();
  private final Trigger operatorRightPadTrigger = operatorController.povRight();
  private final Trigger operatorStartTrigger = operatorController.start();
  private final Trigger operatorBackTrigger = operatorController.back();
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::addReefVisionMeasurement,
                new VisionIOPhoton(
                    VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM),
                new VisionIOPhoton(
                    VisionConstants.LEFT_CAMERA_NAME, VisionConstants.FRONT_LEFT_TRANSFORM),
                new VisionIOLimelight(VisionConstants.LIMELIGHT_NAME, drive::getMegatagRotation));
        elevator = new Elevator(new ElevatorIOTalonFX());
        claw = new Claw(new ClawIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX());
        candle = new CANdleSubsystem(new CANdleIOReal(), drive::getPose);

        LoggedPowerDistribution.getInstance(CANConstants.PDH_ID, ModuleType.kRev);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        swerveSim = new SwerveDriveSimulation(driveSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);

        drive =
            new Drive(
                new GyroIOSim(swerveSim.getGyroSimulation()),
                new ModuleIOMapleSim(swerveSim.getModules()[0]),
                new ModuleIOMapleSim(swerveSim.getModules()[1]),
                new ModuleIOMapleSim(swerveSim.getModules()[2]),
                new ModuleIOMapleSim(swerveSim.getModules()[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::addReefVisionMeasurement,
                new VisionIOSim(
                    VisionConstants.RIGHT_CAMERA_NAME,
                    VisionConstants.FRONT_RIGHT_TRANSFORM,
                    swerveSim::getSimulatedDriveTrainPose),
                new VisionIOSim(
                    VisionConstants.LEFT_CAMERA_NAME,
                    VisionConstants.FRONT_LEFT_TRANSFORM,
                    swerveSim::getSimulatedDriveTrainPose),
                new VisionIOSim(
                    VisionConstants.LOW_BACK_CAMERA_NAME,
                    VisionConstants.LOW_BACK_TRANSFORM,
                    swerveSim::getSimulatedDriveTrainPose));
        claw = new Claw(new ClawIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());
        climber = new Climber(new ClimberIOSim());
        candle = new CANdleSubsystem(new CANdleIO() {}, drive::getPose);
        drive.setPose(new Pose2d(3, 3, Rotation2d.kZero));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::addReefVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        claw = new Claw(new ClawIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        candle = new CANdleSubsystem(new CANdleIO() {}, drive::getPose);
        climber = new Climber(new ClimberIO() {});
        break;
    }

    visualizer = new Visualizer(elevator, wrist, climber);
    superstructure = new Superstructure(elevator, wrist, drive::getPose);

    // Configure PathPlanner commands
    configureNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    /*autoChooser.addOption(
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
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));*/

    SmartDashboard.putString("Composer Input", "1a4");
    SmartDashboard.putBoolean("Use Auto Composer", false);

    SmartDashboard.putData("Home Elevator", Commands.runOnce(() -> superstructure.homeElevator()));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Basic drive controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverLTTrigger.getAsBoolean()));
    driverLeftPadTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverDownPadTrigger.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    // Zonesnap
    driverRTTrigger.whileTrue(new ZoneSnap(drive));

    // Reef Branch alignment
    driverLBTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(
            new BranchAlign(drive, false)
                .alongWith(
                    new AutoElevator(drive::getReefPose, superstructure, () -> selectedScore)))
        .onFalse(Commands.runOnce(() -> superstructure.setState(0), superstructure));
    driverRBTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(
            new BranchAlign(drive, true)
                .alongWith(
                    new AutoElevator(drive::getReefPose, superstructure, () -> selectedScore)))
        .onFalse(Commands.runOnce(() -> superstructure.setState(0), superstructure));

    driverLBTrigger.and(operatorStartTrigger).whileTrue(new BranchAlign(drive, false));
    driverRBTrigger.and(operatorStartTrigger).whileTrue(new BranchAlign(drive, true));

    // Player station alignment
    driverXTrigger.whileTrue(new DriveToPlayerStation(drive, false));
    driverBTrigger.whileTrue(new DriveToPlayerStation(drive, true));

    // Level select & manual
    operatorATrigger.whileTrue(Commands.runOnce(() -> superstructure.setState(2), superstructure));
    operatorBTrigger
        .and(operatorStartTrigger)
        .whileTrue(Commands.runOnce(() -> superstructure.setState(4), superstructure));
    operatorXTrigger
        .and(operatorStartTrigger)
        .whileTrue(Commands.runOnce(() -> superstructure.setState(3), superstructure));
    operatorYTrigger
        .and(operatorStartTrigger)
        .whileTrue(Commands.runOnce(() -> superstructure.setState(5), superstructure));

    operatorBTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(Commands.runOnce(() -> selectedScore = 4, superstructure));
    operatorXTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(Commands.runOnce(() -> selectedScore = 3, superstructure));
    operatorYTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(Commands.runOnce(() -> selectedScore = 5, superstructure));

    // Superstructure home and rehome
    operatorDownPadTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(Commands.runOnce(() -> superstructure.setState(0), superstructure));
    operatorBackTrigger.onTrue(
        Commands.runOnce(() -> superstructure.homeElevator(), superstructure));

    // Intake function
    operatorLBTrigger
        .whileTrue(
            Commands.run(() -> superstructure.setState(1), superstructure)
                .alongWith(claw.runReverse())
                .until(claw::getSensor)
                .andThen(
                    Commands.runOnce(() -> superstructure.setState(0), superstructure)
                        .alongWith(Commands.waitSeconds(0.5).andThen(claw.stopCommand()))))
        .onFalse(claw.stopCommand());

    // Claw controls
    operatorLTTrigger.whileTrue(claw.runReverse()).onFalse(claw.stopCommand());
    operatorRTTrigger.whileTrue(claw.runForward()).onFalse(claw.stopCommand());

    // Algae mode
    operatorRBTrigger.whileTrue(Commands.runOnce(superstructure::algaeMode, superstructure));

    // Processor/Zero Mode
    operatorUpPadTrigger.whileTrue(
        Commands.runOnce(() -> superstructure.setState(8), superstructure));

    // Manual control
    operatorLSTrigger
        .and(operatorStartTrigger)
        .whileTrue(
            Commands.run(
                () -> superstructure.changeElevatorGoal(-operatorController.getLeftY() * 0.005),
                superstructure));
    operatorLSTrigger
        .and(operatorStartTrigger.negate())
        .whileTrue(
            Commands.run(
                () ->
                    superstructure.changeElevatorGoalClamped(
                        -operatorController.getLeftY() * 0.005),
                superstructure));

    operatorRSTrigger.whileTrue(
        Commands.run(
            () -> superstructure.changeWristGoal(-operatorController.getRightY() * 0.01),
            superstructure));

    // Climber
    operatorRightPadTrigger
        .whileTrue(Commands.run(() -> climber.runVoltage(6), climber))
        .onFalse(Commands.runOnce(() -> climber.runVoltage(0)));
    operatorLeftPadTrigger
        .whileTrue(Commands.run(() -> climber.runVoltage(-6), climber))
        .onFalse(Commands.runOnce(() -> climber.runVoltage(0)));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "Superstructure L4",
        Commands.run(() -> superstructure.setState(5), superstructure)
            .until(() -> superstructure.atGoal()));
    NamedCommands.registerCommand("Claw Outtake", claw.runForward());
    NamedCommands.registerCommand("Claw Stop", claw.stopCommand());
    NamedCommands.registerCommand(
        "Stow Elevator",
        Commands.run(() -> superstructure.setState(0), superstructure)
            .until(() -> superstructure.atGoal()));
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

  public Vision getVision() {
    return vision;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Wrist getWrist() {
    return wrist;
  }

  public Claw getClaw() {
    return claw;
  }

  public Visualizer getVisualizer() {
    return visualizer;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public CommandXboxController getDriverController() {
    return driverController;
  }

  public CommandXboxController getOperatorController() {
    return operatorController;
  }

  public Command[] getElevatorCommands() {
    // L1, L2, L3, L4
    return new Command[] {
      new AutoElevator(drive::getPose, superstructure, () -> 2)
          .andThen(Commands.waitUntil(superstructure::atGoal))
          .andThen(new WaitCommand(0.0)),
      new AutoElevator(drive::getPose, superstructure, () -> 3)
          .andThen(Commands.waitUntil(superstructure::atGoal))
          .andThen(new WaitCommand(0.0)),
      new AutoElevator(drive::getPose, superstructure, () -> 4)
          .andThen(Commands.waitUntil(superstructure::atGoal))
          .andThen(new WaitCommand(0.0)),
      new AutoElevator(drive::getPose, superstructure, () -> 5)
          .andThen(Commands.waitUntil(superstructure::atGoal))
          .andThen(new WaitCommand(0.0)),
      Commands.run(() -> superstructure.setState(0), superstructure).withTimeout(1)
    };
  }

  public Command getScoringCommand() {
    return new WaitCommand(0.2).deadlineFor(claw.runForward()).andThen(claw.stopCommand());
  }

  public Command getIntakingCommand() {
    return Commands.run(() -> superstructure.setState(1), superstructure)
        .alongWith(claw.runReverse())
        .until(claw::getSensor)
        .andThen(
            Commands.runOnce(() -> superstructure.setState(0), superstructure)
                .alongWith(Commands.waitSeconds(0.5).andThen(claw.stopCommand())));
  }
}
