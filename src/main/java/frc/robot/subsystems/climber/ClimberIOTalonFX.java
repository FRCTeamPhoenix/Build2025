package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

  private TalonFX climberTalon = new TalonFX(CANConstants.CLIMBER_ID);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final boolean isInverted = false;
  private final boolean brakeMode = true;

  public ClimberIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    climberTalon.getConfigurator().apply(config);

    climberTalon.setPosition(0);

    position = climberTalon.getPosition();
    velocity = climberTalon.getVelocity();
    appliedVolts = climberTalon.getMotorVoltage();
    current = climberTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    climberTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.angle =
        Rotation2d.fromRotations(position.getValueAsDouble() / ClimberConstants.GEAR_RATIO)
            .plus(ClimberConstants.ANGLE_OFFSET);
    inputs.velocityRad = velocity.getValue().in(RadiansPerSecond) / ClimberConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    climberTalon.setControl(new VoltageOut(voltage));
  }
}
