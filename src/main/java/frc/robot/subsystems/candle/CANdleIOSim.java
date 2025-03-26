package frc.robot.subsystems.candle;

public class CANdleIOSim implements CANdleIO {
  private CANdleState state = CANdleState.Off;

  public CANdleIOSim() {
    setMode(CANdleState.Orange);
  }

  @Override
  public void setMode(CANdleState mode) {
    state = mode;
  }

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.mode = state;
  }
}
