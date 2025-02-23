package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.PhoenixUtils.PhoenixGravFF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final ProfiledPIDController controller;
  private final PhoenixGravFF ff;

  private Double setpoint = 0.0;

  private final Alert wristAlert = new Alert("Wrist motor is disconnected", AlertType.kError);

  public Wrist(WristIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        controller =
            new ProfiledPIDController(
                5,
                1,
                1,
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(180), Units.degreesToRadians(120)));
        ff = new PhoenixGravFF(0.05, 0.0, 0.0, 0.17);
        break;
      case SIM:
        controller =
            new ProfiledPIDController(
                10,
                1,
                0.1,
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(30), Units.degreesToRadians(30)));
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.195);
        break;
      default:
        controller =
            new ProfiledPIDController(
                0.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(30), Units.degreesToRadians(30)));
        ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.0);
        break;
    }

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(0.01);
    SmartDashboard.putData(controller);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    if (setpoint != null) {
      Logger.recordOutput("Wrist/Setpoint", setpoint);
      controller.setGoal(setpoint);
      Logger.recordOutput("output", controller.calculate(inputs.angle.getRadians()));
      io.setVoltage(controller.calculate(inputs.angle.getRadians(), setpoint));
      // + ff.calculate(lazyVel, 0, inputs.angle.getRadians()));
    } else {
      Logger.recordOutput("Wrist/Setpoint", -1);
    }
    wristAlert.set(!inputs.connected);
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
