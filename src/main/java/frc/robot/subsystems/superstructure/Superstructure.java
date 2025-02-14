package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist wrist;

  private int superstructureState = 0;
  private boolean manualControl = false;

  private double elevatorSetpoint = 0.0;
  private double wristSetpoint = 0.0;

  private final Supplier<Pose2d> poseSupplier;

  public Superstructure(Elevator elevator, Wrist wrist, Supplier<Pose2d> pose) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.poseSupplier = pose;
  }

  @Override
  public void periodic() {
    elevator.periodic();
    wrist.periodic();

    if (!manualControl) {
      elevatorSetpoint = SuperstructureConstants.ELEVATOR_STATES[superstructureState];
      wristSetpoint = SuperstructureConstants.WRIST_STATES[superstructureState];
      Logger.recordOutput(
          "Superstructure/State", SuperstructureConstants.STATE_NAMES[superstructureState]);
      if (!elevator.atSetpoint()) {
        elevator.runSetpoint(elevatorSetpoint);
        if (elevatorSetpoint - elevator.getHeight() > 0) {
          wrist.setSetpoint(WristConstants.HIGH_MOVE_ANGLE);
        }
        else {
          wrist.setSetpoint(WristConstants.LOW_MOVE_ANGLE);
        }
      } else {
        elevator.runSetpoint(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);
      }
    } else {
      Logger.recordOutput("Superstructure/State", "MANUAL");
      elevator.runSetpoint(elevatorSetpoint);
      wrist.setSetpoint(wristSetpoint);
    }
  }

  public void setState(int state) {
    manualControl = false;
    superstructureState = MathUtil.clamp(state, 0, SuperstructureConstants.STATE_NAMES.length - 1);
  }

  public double getElevatorGoal() {
    return elevatorSetpoint;
  }

  public double getWristGoal() {
    return wristSetpoint;
  }

  public void setElevatorManualGoal(double goal) {
    manualControl = true;
    elevatorSetpoint = MathUtil.clamp(goal, 0, ElevatorConstants.MAX_EXTENSION);
  }

  public void setWristManualGoal(double goal) {
    manualControl = true;
    elevatorSetpoint = MathUtil.clamp(goal, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
  }

  public void changeElevatorGoal(double change) {
    manualControl = true;
    elevatorSetpoint =
        MathUtil.clamp(elevatorSetpoint + change, 0, ElevatorConstants.MAX_EXTENSION);
  }

  public void changeWristGoal(double change) {
    manualControl = true;
    wristSetpoint =
        MathUtil.clamp(wristSetpoint + change, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
  }

  @AutoLogOutput(key = "Superstructure/AtGoal")
  public boolean atGoal() {
    return elevator.atSetpoint() && wrist.atSetpoint();
  }

  public void algaeMode() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d pose = poseSupplier.get();

    List<Pose2d> list =
        isRed
            ? Arrays.asList(PathfindingConstants.ALGAE_RED_POSES)
            : Arrays.asList(PathfindingConstants.ALGAE_BLUE_POSES);

    int[] states =
        isRed ? PathfindingConstants.ALGAE_RED_STATES : PathfindingConstants.ALGAE_BLUE_STATES;

    Pose2d targetPose = pose.nearest(list);

    superstructureState = states[list.indexOf(targetPose)];
  }
}
