package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {

  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(gearbox, 0.04, ClimberConstants.GEAR_RATIO),
          gearbox,
          ClimberConstants.GEAR_RATIO,
          ClimberConstants.ARM_LENGTH,
          ClimberConstants.MIN_ANGLE,
          ClimberConstants.MAX_ANGLE,
          true,
          ClimberConstants.MAX_ANGLE);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sim.update(0.02);
    inputs.angle = Rotation2d.fromDegrees(sim.getAngleRads());
    inputs.velocityRad = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVolts);
  }
}
