package frc.robot.subsystems.candle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CANdleIOSim implements CANdleIO {
  private CANdleState state = CANdleState.Off;

  public CANdleIOSim() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      setMode(CANdleState.Red, true);
    } else {
      setMode(CANdleState.Blue, true);
    }
  }

  @Override
  public void setMode(CANdleState mode, boolean hardSet) {
    state = mode;
  }

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.mode = state;
  }
}
