package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {

  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private Double setpoint = null;

  private final Alert clawAlert = new Alert("Claw motor is disconnected", AlertType.kError);

  public Claw(ClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    if (setpoint != null) {
      io.setVoltage(setpoint);
    }

    clawAlert.set(!inputs.connected);
  }

  public Command runForward() {
    return Commands.runOnce(() -> setpoint = 3.0, this);
  }

  public Command runReverse() {
    return Commands.runOnce(() -> setpoint = -3.0, this);
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> setpoint = 0.0, this);
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void holdPosition() {
    this.setpoint = 0.0;
  }

  public boolean getSensor() {
    return inputs.intakeSensor;
  }
}
