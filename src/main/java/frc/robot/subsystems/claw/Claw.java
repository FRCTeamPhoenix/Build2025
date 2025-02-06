package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {

  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private Double setpoint = null;

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
  }

  public void runForward() {
    this.setpoint = 3.0;
  }

  public void runReverse() {
    this.setpoint = -3.0;
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void holdPosition() {
    this.setpoint = 0.0;
  }
}
