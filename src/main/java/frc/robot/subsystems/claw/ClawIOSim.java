package frc.robot.subsystems.claw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClawConstants;

public class ClawIOSim implements ClawIO {

  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.003, ClawConstants.GEAR_RATIO), gearbox);

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    motorSim.update(0.02);

    inputs.positionRotations = motorSim.getAngularPositionRotations();
    inputs.velocityRotationsPerSec = motorSim.getAngularVelocityRPM();
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = new double[] {motorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    motorSim.setInputVoltage(volts);
  }
}
