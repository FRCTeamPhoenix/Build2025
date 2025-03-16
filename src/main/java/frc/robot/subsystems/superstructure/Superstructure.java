package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import java.util.Arrays;
import java.util.List;
import java.util.function.IntSupplier;
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

  private double lastElevatorSetpoint = 0.0;

  private final Supplier<Pose2d> poseSupplier;

  private boolean atSetpoint = false;

  public Superstructure(Elevator elevator, Wrist wrist, Supplier<Pose2d> pose) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.poseSupplier = pose;
  }

  @Override
  public void periodic() {
    elevator.periodic();
    wrist.periodic();

    if (elevatorSetpoint != SuperstructureConstants.ELEVATOR_STATES[superstructureState]) {
      lastElevatorSetpoint = elevatorSetpoint;
    }

    if (!manualControl) {
      elevatorSetpoint = SuperstructureConstants.ELEVATOR_STATES[superstructureState];
      wristSetpoint = SuperstructureConstants.WRIST_STATES[superstructureState];
      Logger.recordOutput(
          "Superstructure/State", SuperstructureConstants.STATE_NAMES[superstructureState]);
      if (lastElevatorSetpoint == SuperstructureConstants.ELEVATOR_STATES[5]
          && !(Math.abs(elevatorSetpoint - elevator.getHeight()) < 0.05)) {
        wrist.setSetpoint(WristConstants.MOVE_ANGLE);
        if (Math.abs(WristConstants.MOVE_ANGLE - wrist.getAngle()) < 0.02) {
          elevator.runSetpoint(elevatorSetpoint);
        }
      } else {
        wrist.setSetpoint(wristSetpoint);
        elevator.runSetpoint(elevatorSetpoint);
      }
      atSetpoint =
          Math.abs(elevatorSetpoint - elevator.getHeight()) < 0.05
              && Math.abs(wristSetpoint - wrist.getAngle()) < 0.02;

    } else {
      Logger.recordOutput("Superstructure/State", "MANUAL");
      elevator.runSetpoint(elevatorSetpoint);
      wrist.setSetpoint(wristSetpoint);
      atSetpoint = elevator.atSetpoint() && wrist.atSetpoint();
    }
  }

  public void setState(int state) {
    atSetpoint = false;
    manualControl = false;
    superstructureState = MathUtil.clamp(state, 0, SuperstructureConstants.STATE_NAMES.length - 1);
  }

  public void setState(IntSupplier state) {
    atSetpoint = false;
    manualControl = false;
    superstructureState =
        MathUtil.clamp(state.getAsInt(), 0, SuperstructureConstants.STATE_NAMES.length - 1);
  }

  public double getElevatorGoal() {
    return elevatorSetpoint;
  }

  public void homeElevator() {
    manualControl = true;
    elevator.homeElevator();
    elevatorSetpoint = 0;
  }

  public double getWristGoal() {
    return wristSetpoint;
  }

  public double getElevatorHeight() {
    return elevator.getHeight();
  }

  public double getWristAngle() {
    return wrist.getAngle();
  }

  public void setElevatorManualGoal(double goal) {
    manualControl = true;
    elevatorSetpoint = goal;
  }

  public void setWristManualGoal(double goal) {
    manualControl = true;
    elevatorSetpoint = MathUtil.clamp(goal, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
  }

  public void changeElevatorGoal(double change) {
    manualControl = true;
    elevatorSetpoint = elevatorSetpoint + change;
  }

  public void changeElevatorGoalClamped(double change) {
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
    return atSetpoint;
  }

  public void algaeMode() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    Pose2d pose = poseSupplier.get();

    List<Pose2d> list =
        isRed
            ? Arrays.asList(FieldConstants.ALGAE_RED_POSES)
            : Arrays.asList(FieldConstants.ALGAE_BLUE_POSES);

    int[] states = isRed ? FieldConstants.ALGAE_RED_STATES : FieldConstants.ALGAE_BLUE_STATES;

    Pose2d targetPose = pose.nearest(list);

    superstructureState = states[list.indexOf(targetPose)];
  }
}
