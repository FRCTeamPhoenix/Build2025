package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.WristConstants;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private Double setpoint = 0.0;
  private final PIDController controller;
  private final ArmFeedforward ff;

  public Wrist(WristIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        controller = new PIDController(0, 0, 0);
        ff = new ArmFeedforward(0, 0, 0);
        break;
      case SIM:
        controller = new PIDController(2.5, 0.25, 0.01);
        ff = new ArmFeedforward(0, 0.195, 0);
        break;
      default:
        controller = new PIDController(0, 0, 0);
        ff = new ArmFeedforward(0, 0, 0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    if (setpoint != null) {
      Logger.recordOutput("Wrist/Setpoint", setpoint);
      io.setVoltage(controller.calculate(inputs.angleRad, setpoint));
    } else {
      Logger.recordOutput("Wrist/Setpoint", -1);
      io.setVoltage(0.0);
    }
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
  }

  public void goToPosition(IntSupplier positionIndex) {
    this.setpoint =
        SuperstructureConstants.CLAW_ANGLES[
            MathUtil.clamp(
                positionIndex.getAsInt(), 0, SuperstructureConstants.CLAW_ANGLES.length - 1)];
  }

  public double getAngle() {
    return inputs.angleRad;
  }

  public double getSetpoint() {
    return setpoint;
  }
}
