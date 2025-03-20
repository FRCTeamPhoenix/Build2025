package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private Double setpoint = 0.0;
  private boolean killed = false;

  public Wrist(WristIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    if (!killed) {
      if (setpoint != null) {
        Logger.recordOutput("Wrist/Setpoint", setpoint);

        io.setPositionTarget(Rotation2d.fromRadians(setpoint));
      } else {
        Logger.recordOutput("Wrist/Setpoint", -1);
      }
    } else {
      io.setVoltage(0.0);
    }
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
  }

  public double getAngle() {
    return inputs.angle;
  }

  public double getSetpoint() {
    return setpoint;
  }

  @AutoLogOutput(key = "Wrist/AtGoal")
  public boolean atSetpoint() {
    return Math.abs(setpoint - inputs.angle) < 0.01;
  }

  public void runVoltage(double voltage) {
    setpoint = null;
    io.setVoltage(voltage);
  }

  public void stop() {
    io.setVoltage(0);
    setpoint = null;
  }

  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.velocityRad);
  }

  public void kill() {
    killed = true;
    stop();
  }
}
