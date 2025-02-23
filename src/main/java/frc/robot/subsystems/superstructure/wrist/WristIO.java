package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = false;
    public Rotation2d angle = new Rotation2d(0);
    public double velocityRad = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPositionTarget(Rotation2d angle) {}
}
