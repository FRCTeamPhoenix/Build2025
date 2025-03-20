package frc.robot.subsystems.visualizer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Visualizer extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist wrist;

  /*
  private LoggedNetworkNumber x;
  private LoggedNetworkNumber y;
  private LoggedNetworkNumber z;

  private LoggedNetworkNumber roll;
  private LoggedNetworkNumber pitch;
  private LoggedNetworkNumber yaw;*/

  private final LoggedMechanism2d mech = new LoggedMechanism2d(Units.inchesToMeters(28), Units.inchesToMeters(28));
  private final LoggedMechanismRoot2d root = mech.getRoot("root", Units.inchesToMeters(16), 0);
  private final LoggedMechanismLigament2d elevatorMech =
      root.append(
          new LoggedMechanismLigament2d(
              "elevator", ElevatorConstants.MIN_HEIGHT, 90, 10, new Color8Bit(135, 140, 148)));
  private final LoggedMechanismLigament2d wristMech =
      elevatorMech.append(
          new LoggedMechanismLigament2d("wrist", 0.305, 0, 10, new Color8Bit(135, 140, 148)));

  public Visualizer(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    /*
    x = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/X", 0);
    y = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Y", 0);
    z = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Z", 0);

    roll = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Roll", 0);
    pitch = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Pitch", 0);
    yaw = new LoggedNetworkNumber("Mechanism3d/Testing/Tunable/Yaw", 0); */
  }

  @Override
  public void periodic() {
    elevatorMech.setLength(elevator.getHeight() + ElevatorConstants.MIN_HEIGHT);
    wristMech.setAngle(Math.toDegrees(wrist.getAngle()) - 90);

    Logger.recordOutput("Mechanism/2D/Superstructure", mech);

    /*Logger.recordOutput("Mechanism3d/Testing/Zero", Pose3d.kZero);
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
            new Rotation3d(0, -wrist.getAngle(), 0));
    Logger.recordOutput("Mechanism3d/Claw", claw); */
  }
}
