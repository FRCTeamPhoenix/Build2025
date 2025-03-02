package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PhoenixUtils.PhoenixGravFF;

public class ElevatorIOSim implements ElevatorIO {

  public final DCMotor gearbox = DCMotor.getKrakenX60(2);
  public final ElevatorSim elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.GEAR_RATIO,
          ElevatorConstants.PULLEY_RADIUS,
          ElevatorConstants.CARRIAGE_MASS,
          0.0,
          ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT,
          true,
          0.0);

  private double appliedVolts = 0.0;

  private ProfiledPIDController controller =
      new ProfiledPIDController(0.42, 0.730, 0.50, new TrapezoidProfile.Constraints(3, 1.5));
  private PhoenixGravFF ff = new PhoenixGravFF(0.0, 0.0, 0.0, 0.4);

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    inputs.heightMeters = elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.velocityRotationsPerSec =
        elevatorSim.getVelocityMetersPerSecond()
            * ElevatorConstants.GEAR_RATIO
            / (Math.PI * ElevatorConstants.PULLEY_RADIUS)
            / 2;
    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.currentAmps = new double[] {elevatorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
    appliedVolts = voltage;
  }

  @Override
  public void setPositionTarget(double height) {
    controller.setGoal(height);
    appliedVolts = controller.calculate(elevatorSim.getPositionMeters()) + ff.calculate(0, 0, 0);
    elevatorSim.setInputVoltage(appliedVolts);
  }
}
