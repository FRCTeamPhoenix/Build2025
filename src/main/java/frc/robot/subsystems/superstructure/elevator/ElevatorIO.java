package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public double heightMeters = 0;
    public double velocityMetersPerSec = 0;
    public double velocityRotationsPerSec = 0;
    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPositionTarget(double target) {}

  public default void homeElevator() {}
}
