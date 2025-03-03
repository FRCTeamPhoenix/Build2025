package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Double setpoint = 0.0;
  private double offset = 0.0;

  private final Alert elevatorAlert =
      new Alert("Elevator motors are disconnected", AlertType.kError);

  // Mechanism2D
  private final LoggedMechanism2d mech = new LoggedMechanism2d(24, 24);
  private final LoggedMechanismRoot2d root = mech.getRoot("root", 12, 0);
  private final LoggedMechanismLigament2d elevator =
      root.append(
          new LoggedMechanismLigament2d(
              "elevator", ElevatorConstants.MIN_HEIGHT, 90, 10, new Color8Bit(135, 140, 148)));

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Mech2D", mech);
    Logger.recordOutput("Elevator/Offset", offset);

    elevator.setLength(inputs.heightMeters + ElevatorConstants.MIN_HEIGHT - offset);

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
    return inputs.heightMeters;
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
    return Math.abs(setpoint - inputs.heightMeters) < 0.05;
  }
}
