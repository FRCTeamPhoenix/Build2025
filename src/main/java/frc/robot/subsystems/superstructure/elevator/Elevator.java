package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Double setpoint = 0.0;
  private double offset = 0.0;

  private final Alert elevatorAlert =
      new Alert("Elevator motors are disconnected", AlertType.kError);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Offset", offset);

    if (setpoint != null) {
      Logger.recordOutput("Elevator/Setpoint", setpoint);
      Logger.recordOutput("Elevator/Setpoint Offsetted", setpoint + offset);

      io.setPositionTarget(setpoint + offset);
    } else {
      Logger.recordOutput("Elevator/Setpoint", -1.0);
    }

    elevatorAlert.set(!inputs.connected);
  }

  public void runSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public double getHeight() {
    return inputs.heightMeters - offset;
  }

  public void stop() {
    io.setVoltage(0);
    setpoint = null;
  }

  public void homeElevator() {
    offset = inputs.heightMeters;
    setpoint = offset;
  }

  public void runCharacterization(double volts) {
    setpoint = null;
    io.setVoltage(volts);
  }

  public double getFFCharacterizationVelocity() {
    return inputs.velocityRotationsPerSec;
  }

  @AutoLogOutput(key = "Elevator/AtGoal")
  public boolean atSetpoint() {
    return Math.abs(setpoint - (inputs.heightMeters - offset)) < 0.05;
  }
}
