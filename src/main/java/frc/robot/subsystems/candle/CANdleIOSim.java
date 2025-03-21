package frc.robot.subsystems.candle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CANdleIOSim implements CANdleIO {
  private CANdleState state = CANdleState.Off;

  public CANdleIOSim() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      setMode(CANdleState.Red);
    } else {
      setMode(CANdleState.Blue);
    }
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
