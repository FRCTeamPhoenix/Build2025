package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for Maple Sim Gyro */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation sim) {
    this.gyroSimulation = sim;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().baseUnitMagnitude();
  }
}
