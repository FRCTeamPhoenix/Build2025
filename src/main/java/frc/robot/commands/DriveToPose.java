package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  ProfiledPIDController xController =
      new ProfiledPIDController(5, 0, 0, AutoConstants.LINEAR_CONSTRAINTS);
  ProfiledPIDController yController =
      new ProfiledPIDController(5, 0, 0, AutoConstants.LINEAR_CONSTRAINTS);
  ProfiledPIDController angleController =
      new ProfiledPIDController(3, 0, 0.2, AutoConstants.ANGLE_CONSTRAINTS);

  final Drive drive;
  final Supplier<Pose2d> robotPose;
  Pose2d target;
  boolean isDone = false;

  public DriveToPose(Drive drive, Pose2d targetPose, Supplier<Pose2d> robotPose) {
    this.drive = drive;
    this.target = targetPose;
    this.robotPose = robotPose;

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setGoal(target.getX());
    yController.setGoal(target.getY());
    angleController.setGoal(target.getRotation().getRadians());
    xController.setTolerance(0.008);
    yController.setTolerance(0.008);
    angleController.setTolerance(0.01);

    Logger.recordOutput("PoseAlignment/Target", target);

    super.addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset(robotPose.get().getX());
    yController.reset(robotPose.get().getY());
    angleController.reset(robotPose.get().getRotation().getRadians());
    isDone = false;
    Logger.recordOutput("PoseAlignment/AtGoal", false);
  }

  @Override
  public void execute() {
    Pose2d currentPosition = robotPose.get();

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(currentPosition.getX()),
            yController.calculate(currentPosition.getY()),
            angleController.calculate(currentPosition.getRotation().getRadians()),
            drive.getRotation());

    Pose2d setpoint =
        new Pose2d(
            xController.getSetpoint().position,
            yController.getSetpoint().position,
            Rotation2d.fromRadians(angleController.getSetpoint().position));

    Pose2d[] lazyTrajectory = new Pose2d[] {currentPosition, setpoint, target};

    Logger.recordOutput("PoseAlignment/Speeds", speeds);
    Logger.recordOutput("PoseAlignment/LazyTrajectory", lazyTrajectory);

    drive.runVelocity(speeds);
    isDone = xController.atGoal() && yController.atGoal() && angleController.atGoal();

    Logger.recordOutput(
        "PoseAlignment/AtGoal",
        xController.atGoal() && yController.atGoal() && angleController.atGoal());
    Logger.recordOutput("PoseAlignment/ControllerStates/XController", xController.atGoal());
    Logger.recordOutput("PoseAlignment/ControllerStates/YController", yController.atGoal());
    Logger.recordOutput("PoseAlignment/ControllerStates/AngleController", angleController.atGoal());
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  public void setNewTarget(Pose2d target) {
    this.target = target;

    xController.setGoal(target.getX());
    yController.setGoal(target.getY());
    angleController.setGoal(target.getRotation().getRadians());
    isDone = false;
    Logger.recordOutput("PoseAlignment/Target", target);
  }

  public void setNewConstraints(Constraints linear, Constraints angular) {
    xController.setConstraints(linear);
    yController.setConstraints(linear);
    angleController.setConstraints(angular);
  }
}
