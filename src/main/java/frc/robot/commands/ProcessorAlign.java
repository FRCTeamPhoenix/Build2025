package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.drive.Drive;

public class ProcessorAlign extends DriveToPose {

  public ProcessorAlign(Drive drive) {
    super(drive, new Pose2d());
  }

  @Override
  public void initialize() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    setNewTarget(isRed ? PathfindingConstants.RED_PROCESSOR : PathfindingConstants.BLUE_PROCESSOR);
    super.initialize();
  }
}
