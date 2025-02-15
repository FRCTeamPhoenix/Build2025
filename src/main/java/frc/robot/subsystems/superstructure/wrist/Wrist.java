package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.PhoenixUtils.PhoenixGravFF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final PIDController controller;
  private final PhoenixGravFF ff;

  private Double setpoint = 0.0;

  private final Alert wristAlert = new Alert("Wrist  bE motor is disconnected", AlertType.kError);

  public Wrist(WristIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        controller = new PIDController(2.0, 0.15, 0.01);
        ff = new PhoenixGravFF(0.15763, 0.0, 0.0, 0.15);
        break;
      case SIM:
        controller = new PIDController(10, 0.15, 0.01);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.195);
        break;
      default:
        controller = new PIDController(0, 0, 0);
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.0);
        break;
    }

    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    if (setpoint != null) {
      Logger.recordOutput("Wrist/Setpoint", setpoint);
      double lazyVel = 0;
      if (!controller.atSetpoint()) {
        lazyVel = setpoint - inputs.angle.getRadians();
      }
      io.setVoltage(
          controller.calculate(inputs.angle.getRadians(), setpoint)
              + ff.calculate(lazyVel, 0, inputs.angle.getRadians()));
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
    setpoint = null;
    io.setVoltage(voltage);
  }

  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.velocityRad);
  }
}
