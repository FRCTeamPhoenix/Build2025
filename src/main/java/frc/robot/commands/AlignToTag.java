package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.photon.PhotonIO.TargetObservation;

public class AlignToTag extends Command {

    //TODO: Tune PIDs for real bot, add PIDs for sim bot
    PIDController turnController = new PIDController(0.01, 0, 0);
    PIDController strafeController = new PIDController(1.08, 0.6, 0.1);

    Photon photonSubsystem;
    int cameraID;
    Drive drive;

    public AlignToTag(int cameraID, Photon photonSubsystem, Drive drive) {
        this.cameraID = cameraID;
        this.photonSubsystem = photonSubsystem;
        this.drive = drive;
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        super.addRequirements(photonSubsystem, drive);
    }

    @Override
    public void execute() {
        TargetObservation target = photonSubsystem.getBestTarget(cameraID);

        Pose3d pose = Pose3d.kZero;
        pose = pose.transformBy(VisionConstants.CAMERA_TRANSFORMS[cameraID]);
        Pose3d tagPose = pose.transformBy(target.cameraToTarget());

        Pose3d drivePose = new Pose3d(drive.getPose());
        drivePose = drivePose.transformBy(VisionConstants.CAMERA_TRANSFORMS[cameraID]);
        Pose3d realTagPose = drivePose.transformBy(target.cameraToTarget());

        Logger.recordOutput("TagAlignment/Target", realTagPose);
        double yDifference = target.cameraToTarget().getY();
        double thetaDifference = tagPose.getRotation().minus(pose.getRotation()).getZ();
        Logger.recordOutput("TagAlignment/Y", yDifference);
        Logger.recordOutput("TagAlignment/Theta", thetaDifference);
    

        ChassisSpeeds speeds = new ChassisSpeeds(0, strafeController.calculate(-yDifference, 0),
                turnController.calculate(thetaDifference, 0));
                Logger.recordOutput("TagAlignment/Speeds", speeds);
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        turnController.close();
        strafeController.close();
    }

    @Override
    public boolean isFinished() {
        TargetObservation target = photonSubsystem.getBestTarget(cameraID);
        
        double angle = MathUtil.applyDeadband(target.tx().getDegrees(), 0.01);
        double y = MathUtil.applyDeadband(target.cameraToTarget().getY(), 0.05);

        if (angle == 0 && y == 0) {
            return true;
        }
        return false;
    }
}
