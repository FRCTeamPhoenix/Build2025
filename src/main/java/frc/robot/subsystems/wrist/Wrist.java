package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private Double setpoint = 0.0;
  private final double kG = 0.475;
  private final PIDController controller;
  private final ArmFeedforward ff;

  public Wrist(WristIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        controller = new PIDController(1.21, 0.0452, 0.019);
        ff = new ArmFeedforward(0, 0, 0);
        break;
      case SIM:
        controller = new PIDController(2.5, 0.245, 0.01);
        ff = new ArmFeedforward(0, 0.195, 0);
        break;
      default:
        controller = new PIDController(0, 0, 0);
        ff = new ArmFeedforward(0, 0, 0);
        break;
    }
    controller.enableContinuousInput(-Math.PI, -Math.PI);
    SmartDashboard.putData(controller);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    if (setpoint != null) {
      Logger.recordOutput("Wrist/Setpoint", setpoint);
      io.setVoltage(controller.calculate(inputs.angle.getRadians(), setpoint) + kg());
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

  public void runVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double kg() {
    return kG * Math.cos(inputs.angle.getRadians());
  }
}
