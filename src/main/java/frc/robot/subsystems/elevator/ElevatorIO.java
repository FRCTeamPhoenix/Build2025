package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double height = 0;
    public double velocity = 0;
    public double appliedVolts = 0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
