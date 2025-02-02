package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

    public final DCMotor gearbox = DCMotor.getKrakenX60(2);
    public final ElevatorSim elevatorSim = new ElevatorSim(gearbox, ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.PULLEY_RADIUS, ElevatorConstants.CARRIAGE_MASS,
            0.0, ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT, true, 0.0);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(0.02);
        inputs.heightMeters = elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.velocityRotationsPerSec = elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.GEAR_RATIO / (Math.PI * ElevatorConstants.PULLEY_RADIUS) / 2;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {elevatorSim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
        appliedVolts = voltage;
    }
}
