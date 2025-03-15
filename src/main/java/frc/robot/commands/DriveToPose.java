package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  ProfiledPIDController angleController =
      new ProfiledPIDController(3, 0, 0.2, AutoConstants.ANGLE_CONSTRAINTS);

  HolonomicDriveController controller =
      new HolonomicDriveController(xController, yController, angleController);

  final Drive drive;
  final Supplier<Pose2d> robotPose;
  Pose2d target;
  boolean isDone = false;

  public DriveToPose(Drive drive, Pose2d targetPose, Supplier<Pose2d> robotPose) {
    this.drive = drive;
    this.target = targetPose;
    this.robotPose = robotPose;

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // xController.setGoal(target.getX());
    // yController.setGoal(target.getY());
    // angleController.setGoal(target.getRotation().getRadians());
    xController.setTolerance(0.008);
    yController.setTolerance(0.008);
    angleController.setTolerance(0.01);

    Logger.recordOutput("PoseAlignment/Target", target);

    super.addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    angleController.reset(robotPose.get().getRotation().getRadians());
    isDone = false;
    Logger.recordOutput("PoseAlignment/AtGoal", false);
  }

  @Override
  public void execute() {
    Pose2d currentPosition = robotPose.get();

    // ChassisSpeeds speeds =
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         xController.calculate(currentPosition.getX()),
    //         yController.calculate(currentPosition.getY()),
    //         angleController.calculate(currentPosition.getRotation().getRadians()),
    //         drive.getRotation());

    ChassisSpeeds speeds = controller.calculate(currentPosition, target, 0, target.getRotation());

    drive.runVelocity(speeds);
    isDone = controller.atReference();

    Logger.recordOutput("PoseAlignment/AtGoal", controller.atReference());
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

    // xController.setGoal(target.getX());
    // yController.setGoal(target.getY());
    // angleController.setGoal(target.getRotation().getRadians());
    isDone = false;
    Logger.recordOutput("PoseAlignment/Target", target);
  }

  public void setNewPValues(double strafeP, double rotationP) {
    xController.setP(strafeP);
    yController.setP(strafeP);
    angleController.setP(rotationP);
  }
}
