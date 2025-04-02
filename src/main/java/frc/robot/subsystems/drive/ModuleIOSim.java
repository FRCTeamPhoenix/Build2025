package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Maple sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation sim;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  public ModuleIOSim(SwerveModuleSimulation module) {
    sim = module;
    this.driveMotor = sim.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
    this.turnMotor = sim.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.drivePositionRad = sim.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = sim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = sim.getDriveMotorAppliedVoltage().magnitude();
    inputs.driveCurrentAmps = Math.abs(sim.getDriveMotorSupplyCurrent().magnitude());

    inputs.turnConnected = true;
    inputs.turnPosition = sim.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = sim.getSteerRelativeEncoderVelocity().magnitude();
    inputs.turnAppliedVolts = sim.getSteerMotorAppliedVoltage().magnitude();
    inputs.turnCurrentAmps = Math.abs(sim.getSteerMotorSupplyCurrent().magnitude());
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.requestVoltage(Volts.of(MathUtil.clamp(volts, -12.0, 12.0)));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.requestVoltage(Volts.of(MathUtil.clamp(volts, -12.0, 12.0)));
  }
}
