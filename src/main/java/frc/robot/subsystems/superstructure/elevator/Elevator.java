package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Double setpoint = 0.0;
  private double offset = 0.0;
  private boolean killed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Offset", offset);

    if (!killed) {
      if (setpoint != null) {
        Logger.recordOutput("Elevator/Setpoint", setpoint);
        Logger.recordOutput("Elevator/Setpoint Offsetted", setpoint + offset);

        io.setPositionTarget(setpoint + offset);
      } else {
        Logger.recordOutput("Elevator/Setpoint", -1.0);
      }
    } else {
      io.setVoltage(0.0);
    }
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
    // offset = inputs.heightMeters;
    // setpoint = offset;
    io.rehomeElevator();
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

  public void kill() {
    killed = true;
    stop();
  }

  public void revive() {
    killed = false;
    setpoint = getHeight();
  }
}
