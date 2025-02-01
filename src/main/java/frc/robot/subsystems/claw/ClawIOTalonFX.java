package frc.robot.subsystems.claw;

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
import frc.robot.Constants.ClawConstants;

public class ClawIOTalonFX implements ClawIO {

  private TalonFX clawTalon = new TalonFX(ClawConstants.CLAW_ID);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final boolean isInverted = true;

  public ClawIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    clawTalon.getConfigurator().apply(config);
    setBrakeMode(true);

    position = clawTalon.getPosition();
    velocity = clawTalon.getVelocity();
    appliedVolts = clawTalon.getMotorVoltage();
    current = clawTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    clawTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.positionRotations = position.getValueAsDouble() / ClawConstants.GEAR_RATIO;
    inputs.velocityRotationsPerSec = velocity.getValueAsDouble() / ClawConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double voltage) {
    clawTalon.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    clawTalon.getConfigurator().apply(config);
  }
}
