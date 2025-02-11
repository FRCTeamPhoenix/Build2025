package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist wrist;

  private int superstructureState = 0;
  private boolean manualControl = false;

  private double elevatorSetpoint = 0.0;
  private double wristSetpoint = 0.0;

  public Superstructure(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {
    if (!manualControl) {
      elevatorSetpoint = SuperstructureConstants.elevatorStates[superstructureState];
      wristSetpoint = SuperstructureConstants.wristStates[superstructureState];
      Logger.recordOutput(
          "Superstructure/State", SuperstructureConstants.stateNames[superstructureState]);
      if (!elevator.atSetpoint()) {
        elevator.runSetpoint(elevatorSetpoint);
        wrist.setSetpoint(WristConstants.MOVE_ANGLE);
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
    superstructureState = MathUtil.clamp(state, 0, SuperstructureConstants.stateNames.length - 1);
  }

  public void cycleState(int change) {
    manualControl = false;
    superstructureState =
        MathUtil.clamp(
            superstructureState + change, 0, SuperstructureConstants.stateNames.length - 1);
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
}
