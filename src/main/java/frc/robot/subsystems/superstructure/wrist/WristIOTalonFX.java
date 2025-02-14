package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {

  private TalonFX wristTalon = new TalonFX(CANConstants.WRIST_ID);
  private Canandmag encoder = new Canandmag(CANConstants.WRIST_ENCODER_ID);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final boolean isInverted = true;
  private final boolean brakeMode = true;

  public WristIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    wristTalon.getConfigurator().apply(config);

    wristTalon.setPosition(encoder.getAbsPosition() * WristConstants.GEAR_RATIO);

    position = wristTalon.getPosition();
    velocity = wristTalon.getVelocity();
    appliedVolts = wristTalon.getMotorVoltage();
    current = wristTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    wristTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    if (encoder.isConnected(0.03)) {
      inputs.angle = Rotation2d.fromRotations(encoder.getAbsPosition()).plus(Rotation2d.kZero);
      wristTalon.setPosition(encoder.getAbsPosition() * WristConstants.GEAR_RATIO);
    } else {
      inputs.angle =
          Rotation2d.fromRotations(position.getValueAsDouble() / WristConstants.GEAR_RATIO)
              .plus(Rotation2d.kZero);
    }
    inputs.velocityRad = velocity.getValue().in(RadiansPerSecond) / WristConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    wristTalon.setControl(new VoltageOut(voltage));
  }
}
