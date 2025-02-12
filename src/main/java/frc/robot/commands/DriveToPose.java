package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  ProfiledPIDController xController =
      new ProfiledPIDController(5, 0, 0, PathfindingConstants.LINEAR_CONSTRAINTS);
  ProfiledPIDController yController =
      new ProfiledPIDController(5, 0, 0, PathfindingConstants.LINEAR_CONSTRAINTS);
  ProfiledPIDController angleController =
      new ProfiledPIDController(3, 0, 0.2, PathfindingConstants.ANGLE_CONSTRAINTS);

  Drive drive;
  Pose2d target;

  public DriveToPose(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.target = targetPose;

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setGoal(target.getX());
    yController.setGoal(target.getY());
    angleController.setGoal(target.getRotation().getRadians());

    Logger.recordOutput("PoseAlignment/Target", target);

    super.addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset(drive.getPose().getX());
    yController.reset(drive.getPose().getY());
    angleController.reset(drive.getPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPosition = drive.getPose();

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

    Pose2d[] roughTrajectory = new Pose2d[] {currentPosition, setpoint, target};

    Logger.recordOutput("PoseAlignment/Speeds", speeds);
    Logger.recordOutput("PoseAlignment/RoughTrajectory", roughTrajectory);

    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && angleController.atGoal();
  }

  public void setNewTarget(Pose2d target, Rotation2d rotationOffset) {
    this.target = target;

    xController.setGoal(target.getX());
    yController.setGoal(target.getY());
    angleController.setGoal(target.getRotation().plus(rotationOffset).getRadians());

    Logger.recordOutput("PoseAlignment/Target", target);
  }

  public void setNewConstraints(Constraints linear, Constraints angular) {
    xController.setConstraints(linear);
    yController.setConstraints(linear);
    angleController.setConstraints(angular);
  }
}
