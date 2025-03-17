package frc.robot.subsystems.candle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.candle.CANdleIO.CANdleState;
import org.littletonrobotics.junction.Logger;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdleIO candle;
  private final CANdleIOInputsAutoLogged inputs = new CANdleIOInputsAutoLogged();

  public CANdleSubsystem(CANdleIO io) {
    this.candle = io;
  }

  @Override
  public void periodic() {
    candle.updateInputs(inputs);
    Logger.processInputs("CANDle", inputs);
  }

  public void setState(CANdleState state) {
    candle.setMode(state);
  }
}
