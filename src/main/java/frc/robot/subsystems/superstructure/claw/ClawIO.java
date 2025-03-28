package frc.robot.subsystems.superstructure.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean intakeSensor = false;
  }

  public default void updateInputs(ClawIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setBrakeMode(boolean enabled) {}
}
