package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double heightMeters = 0;
        public double velocityMetersPerSec = 0;
        public double velocityRotationsPerSec = 0;
        public double appliedVolts = 0;
        public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
}
