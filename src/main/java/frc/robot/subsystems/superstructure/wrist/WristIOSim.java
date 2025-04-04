package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {

  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(gearbox, 0.04, WristConstants.GEAR_RATIO),
          gearbox,
          WristConstants.GEAR_RATIO,
          WristConstants.CLAW_LENGTH,
          WristConstants.MIN_ANGLE,
          WristConstants.MAX_ANGLE,
          true,
          WristConstants.MAX_ANGLE);
  private double appliedVolts = 0.0;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          5,
          1,
          1,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(900), Units.degreesToRadians(900)));

  @Override
  public void updateInputs(WristIOInputs inputs) {
    sim.update(0.02);
    inputs.angle = sim.getAngleRads();
    inputs.velocityRad = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPositionTarget(Rotation2d angle) {
    controller.setGoal(angle.getRadians());
    appliedVolts = controller.calculate(sim.getAngleRads());
    sim.setInputVoltage(appliedVolts);
  }
}
