package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

    private TalonFX elevatorTalon = new TalonFX(CANConstants.ELEVATOR_ID);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    private final boolean isInverted = true;
    private final boolean brakeEnabled = false;

    public ElevatorIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorTalon.getConfigurator().apply(config);

        var motorConfig = new MotorOutputConfigs();
        motorConfig.Inverted = isInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        motorConfig.NeutralMode = brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        elevatorTalon.getConfigurator().apply(config);

        position = elevatorTalon.getPosition();
        velocity = elevatorTalon.getVelocity();
        appliedVolts = elevatorTalon.getMotorVoltage();
        current = elevatorTalon.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVolts,
                current);
        elevatorTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVolts,
                current);
        inputs.heightMeters = position.getValueAsDouble()
                / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.PULLEY_RADIUS;
        inputs.velocityMetersPerSec = velocity.getValueAsDouble()
                / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.PULLEY_RADIUS;        
        inputs.velocityRotationsPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = new double[] { current.getValueAsDouble() };
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorTalon.setControl(new VoltageOut(voltage));
    }
}
