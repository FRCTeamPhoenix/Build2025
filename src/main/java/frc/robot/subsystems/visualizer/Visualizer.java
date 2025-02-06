package frc.robot.subsystems.visualizer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Visualizer extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist wrist;

  private LoggedNetworkNumber x;
  private LoggedNetworkNumber y;
  private LoggedNetworkNumber z;

  private LoggedNetworkNumber roll;
  private LoggedNetworkNumber pitch;
  private LoggedNetworkNumber yaw;

  private double climberSetpoint = Math.PI / 4;
  private boolean climberGoingUp = true;

  public Visualizer(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    x = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/X", 0);
    y = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Y", 0);
    z = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Z", 0);

    roll = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Roll", 0);
    pitch = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Pitch", 0);
    yaw = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Yaw", 0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Mechanism3d/Testing/Zero", Pose3d.kZero);
    Pose3d tunable =
        new Pose3d(x.get(), y.get(), z.get(), new Rotation3d(roll.get(), pitch.get(), yaw.get()));
    Logger.recordOutput("Mechanism3d/Testing/TunablePose", tunable);

    // Elevator
    double height = elevator.getHeight();
    Pose3d firstStage = new Pose3d(0, 0, height / 2 + 0.070668, Rotation3d.kZero);
    Pose3d secondStage = new Pose3d(0, 0, height + 0.070668, Rotation3d.kZero);
    Logger.recordOutput("Mechanism3d/Elevator/FirstStage", firstStage);
    Logger.recordOutput("Mechanism3d/Elevator/SecondStage", secondStage);

    double elevatorExtension = ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT;
    Pose3d claw =
        new Pose3d(
            0.178779,
            0,
            height + 0.235 + 0.070668 + (height / elevatorExtension * 0.5),
            new Rotation3d(0, -wrist.getAngle() + Math.PI / 2, 0));
    Logger.recordOutput("Mechanism3d/Claw", claw);

    // Climber
    if (climberGoingUp) {
      climberSetpoint = MathUtil.clamp(climberSetpoint - 0.05, -Math.PI / 4, Math.PI / 4);
    } else {
      climberSetpoint = MathUtil.clamp(climberSetpoint + 0.05, -Math.PI / 4, Math.PI / 4);
    }

    if (Math.abs(climberSetpoint) == Math.PI / 4) {
      climberGoingUp = !climberGoingUp;
    }
    Pose3d climber =
        new Pose3d(0, 0.276527, 0.417863 + 0.070668, new Rotation3d(climberSetpoint, 0, 0));
    Logger.recordOutput("Mechanism3d/Climber", climber);
  }
}
