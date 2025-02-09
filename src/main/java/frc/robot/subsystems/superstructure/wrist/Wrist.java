package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.wrist.WristIOInputsAutoLogged;
import frc.robot.util.PhoenixUtils.PhoenixGravFF;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final PIDController controller;
  private final PhoenixGravFF ff;

  private Double setpoint = 0.0;

  public Wrist(WristIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        controller = new PIDController(1.21, 0.0452, 0.019);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.475);
        break;
      case SIM:
        controller = new PIDController(10, 0.245, 0.01);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.195);
        break;
      default:
        controller = new PIDController(0, 0, 0);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.0);
        break;
    }

    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    if (setpoint != null) {
      Logger.recordOutput("Wrist/Setpoint", setpoint);
      io.setVoltage(
          controller.calculate(inputs.angle.getRadians(), setpoint)
              + ff.calculate(0, 0, inputs.angle.getRadians()));
    } else {
      Logger.recordOutput("Wrist/Setpoint", -1);
    }
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
  }

  public double getAngle() {
    return inputs.angle.getRadians();
  }

  public double getSetpoint() {
    return setpoint;
  }

  @AutoLogOutput(key = "Wrist/AtGoal")
  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  public void runVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}
