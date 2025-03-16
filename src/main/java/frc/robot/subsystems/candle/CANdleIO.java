package frc.robot.subsystems.candle;

import org.littletonrobotics.junction.AutoLog;

public interface CANdleIO {
  @AutoLog
  public static class CANdleIOInputs {
    // public double appliedVolts = 0;
    public CANdleState mode = CANdleState.Off;
    public double[] busVolts = {0, 0};
    public double[] railVolts = {0, 0};
  }

  public enum CANdleState {
  FireAnimation,
  RainbowAnimation,
  RgbFadeAnimation,
  Red,
  Blue,
  Green,
  Orange,
  //PhoenixOrange,
  //PhoenixYellow,
  PhoenixRed,
  Off;
  }

  public default void updateInputs(CANdleIOInputs inputs) {}

  public default void setMode(CANdleState mode) {}
}