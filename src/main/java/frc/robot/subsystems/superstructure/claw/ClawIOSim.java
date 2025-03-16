package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClawConstants;

public class ClawIOSim implements ClawIO {

  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.003, ClawConstants.GEAR_RATIO), gearbox);

  public ClawIOSim() {
    SmartDashboard.putBoolean("ClawSensorSim", false);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    motorSim.update(0.02);

    inputs.intakeSensor = SmartDashboard.getBoolean("ClawSensorSim", false);

    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    motorSim.setInputVoltage(volts);
  }
}
