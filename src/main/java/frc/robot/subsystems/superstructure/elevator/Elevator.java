package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PhoenixUtils.PhoenixGravFF;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController pidController;
  private final PhoenixGravFF feedforward;

  private Double setpoint = 0.0;

  // Mechanism2D
  private final LoggedMechanism2d mech = new LoggedMechanism2d(24, 24);
  private final LoggedMechanismRoot2d root = mech.getRoot("root", 12, 0);
  private final LoggedMechanismLigament2d elevator =
      root.append(
          new LoggedMechanismLigament2d(
              "elevator", ElevatorConstants.MIN_HEIGHT, 90, 10, new Color8Bit(135, 140, 148)));

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.CURRENT_MODE) {
      case REAL:
        pidController =
            new ProfiledPIDController(11, 1, 0.0, new TrapezoidProfile.Constraints(1.5, 0.5));
        feedforward = new PhoenixGravFF(0.316, 0.506, 0.0, 0.565);
        break;
      case SIM:
        pidController =
            new ProfiledPIDController(0.42, 0.730, 0.50, new TrapezoidProfile.Constraints(3, 3));
        feedforward = new PhoenixGravFF(0.0, 0.0, 0.0, 0.4);
        break;
      default:
        pidController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3, 3));
        feedforward = new PhoenixGravFF(0.0, 0.0, 0.0, 0.0);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Mech2D", mech);

    elevator.setLength(inputs.heightMeters + ElevatorConstants.MIN_HEIGHT);

    if (setpoint != null) {
      Logger.recordOutput("Elevator/Setpoint", setpoint);

      pidController.setGoal(setpoint);

      io.setVoltage(
          pidController.calculate(inputs.heightMeters)
              + feedforward.calculate(pidController.getSetpoint().velocity, 0, 0));
    } else {
      Logger.recordOutput("Elevator/Setpoint", -1.0);
    }
  }

  public void goToPosition(IntSupplier positionIndex) {
    this.setpoint = ElevatorConstants.POSITIONS[positionIndex.getAsInt()];
  }

  public void runSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, 0, ElevatorConstants.MAX_EXTENSION);
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public void stop() {
    io.setVoltage(feedforward.calculate(0, 0, 0));
    setpoint = null;
  }

  public void runCharacterization(double volts) {
    setpoint = null;
    io.setVoltage(volts);
  }

  public double getFFCharacterizationVelocity() {
    return inputs.velocityRotationsPerSec;
  }

  @AutoLogOutput(key = "Elevator/AtGoal")
  public boolean atSetpoint() {
    return pidController.atGoal();
  }
}
