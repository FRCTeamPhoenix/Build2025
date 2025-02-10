package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.PhoenixUtils.PhoenixGravFF;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final PIDController controller;
  private final PhoenixGravFF ff;

  private Double setpoint = 0.0;

  public Climber(ClimberIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        controller = new PIDController(0.0, 0.0, 0.0);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.0);
        break;
      case SIM:
        controller = new PIDController(1.0, 0.0, 0.0);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.0);
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
    Logger.processInputs("Climber", inputs);
    if (setpoint != null) {
      Logger.recordOutput("Climber/Setpoint", setpoint);
      io.setVoltage(
          controller.calculate(inputs.angle.getRadians(), setpoint)
              + ff.calculate(0, 0, inputs.angle.getRadians()));
    } else {
      Logger.recordOutput("Climber/Setpoint", -1);
    }
  }

  public void setSetpoint(double setpoint) {
    this.setpoint =
        MathUtil.clamp(setpoint, ClimberConstants.MIN_ANGLE, ClimberConstants.MAX_ANGLE);
  }

  public void changeSetpoint(double change) {
    this.setpoint =
        MathUtil.clamp(setpoint + change, ClimberConstants.MIN_ANGLE, ClimberConstants.MAX_ANGLE);
  }

  public double getAngle() {
    return inputs.angle.getRadians();
  }

  public double getSetpoint() {
    return setpoint;
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  public void runVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}
