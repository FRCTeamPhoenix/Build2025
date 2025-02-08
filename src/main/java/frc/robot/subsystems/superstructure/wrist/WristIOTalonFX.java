package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {

  private TalonFX wristTalon = new TalonFX(CANConstants.WRIST_ID);
  private SparkMax sparkMax = new SparkMax(CANConstants.WRIST_SPARK, MotorType.kBrushless);
  private SparkAbsoluteEncoder encoder = sparkMax.getAbsoluteEncoder();

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

    velocity = wristTalon.getVelocity();
    appliedVolts = wristTalon.getMotorVoltage();
    current = wristTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    wristTalon.optimizeBusUtilization();

    encoder.getPosition();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.angle =
        Rotation2d.fromRotations(encoder.getPosition()).plus(WristConstants.ANGLE_OFFSET);
    inputs.velocityRad = velocity.getValue().in(RadiansPerSecond) / WristConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    wristTalon.setControl(new VoltageOut(voltage));
  }
}
