package frc.robot.subsystems.candle;

import org.littletonrobotics.junction.AutoLog;

public interface CANdleIO {
  @AutoLog
  public static class CANdleIOInputs {
    // public double appliedVolts = 0;
    public CANdleState mode = CANdleState.Off;
    public double busVolts = 0;
    public double railVolts = 0;
  }

  public default void updateInputs(CANdleIOInputs inputs) {}

  public default void setMode(CANdleState mode) {}
}

enum CANdleState {
  FireAnimation,
  RainbowAnimation,
  RgbFadeAnimation,
  Red,
  Blue,
  Green,
  Orange,
  Off,
  //PhoenixOrange,
  //PhoenixYellow,
  PhoenixRed;
}
