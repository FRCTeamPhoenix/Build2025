package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double angleRad = 0;
    public double velocityRad = 0;
    public double appliedVolts = 0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
