package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.photon.PhotonIO.TargetObservation;

public class AlignToTag extends Command {

    PIDController turnController = new PIDController(0.1, 0, 0.1);
    PIDController strafeController = new PIDController(0.1, 0, 0.1);

    Photon photonSubsystem;
    int cameraID;
    int tagID;
    Drive drive;

    public AlignToTag(int cameraID, int tagID, Photon photonSubsystem, Drive drive) {
        this.cameraID = cameraID;
        this.photonSubsystem = photonSubsystem;
        this.drive = drive;
        this.tagID = tagID;

        super.addRequirements(photonSubsystem, drive);
    }

    @Override
    public void execute() {
        TargetObservation target = photonSubsystem.getTag(cameraID, tagID);
        ChassisSpeeds speeds = new ChassisSpeeds(0, strafeController.calculate(target.cameraToTarget().getY(), 0),
                turnController.calculate(target.tx().getDegrees(), 0));
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        turnController.close();
        strafeController.close();
    }

    @Override
    public boolean isFinished() {
        TargetObservation target = photonSubsystem.getTag(cameraID, tagID);
        
        double angle = MathUtil.applyDeadband(target.tx().getDegrees(), 0.1);
        double y = MathUtil.applyDeadband(target.cameraToTarget().getY(), 0.1);

        if (angle == 0 && y == 0) {
            return true;
        }
        return false;
    }
}
