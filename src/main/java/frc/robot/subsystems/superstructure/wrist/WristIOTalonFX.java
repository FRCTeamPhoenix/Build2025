package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {

  private TalonFX wristTalon = new TalonFX(CANConstants.WRIST_ID);
  private Canandmag encoder = new Canandmag(CANConstants.WRIST_ENCODER_ID);

  private final Alert wristAlert = new Alert("Wrist motor is disconnected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Wrist encoder is disconnected", AlertType.kWarning);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final boolean isInverted = false;
  private final boolean brakeMode = true;
  private boolean hasReset = false;
  final MotionMagicVoltage request = new MotionMagicVoltage(0);

  public WristIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    var slot0Configs = config.Slot0;
    slot0Configs.kP = 5; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Units.degreesToRotations(1260)
            * WristConstants.GEAR_RATIO; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        Units.degreesToRotations(1160)
            * WristConstants.GEAR_RATIO; // Target acceleration of 160 rps/s (0.5 seconds)
    wristTalon.getConfigurator().apply(config);

    position = wristTalon.getPosition();
    velocity = wristTalon.getVelocity();
    appliedVolts = wristTalon.getMotorVoltage();
    current = wristTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    wristTalon.optimizeBusUtilization();

    // encoder.setSettings(new CanandmagSettings());
    encoderAlert.set(!encoder.isConnected());
    wristTalon.setPosition(
        Rotation2d.fromRotations(encoder.getAbsPosition()).minus(Rotation2d.kZero).getRotations()
            * WristConstants.GEAR_RATIO);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (!hasReset) {
      wristTalon.setPosition(
          Rotation2d.fromRotations(encoder.getAbsPosition()).minus(Rotation2d.kZero).getRotations()
              * WristConstants.GEAR_RATIO);
      hasReset = true;
    }

    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.connected = status.isOK();
    wristAlert.set(!inputs.connected);

    encoderAlert.set(!encoder.isConnected());

    inputs.angle = position.getValue().in(Radian) / WristConstants.GEAR_RATIO;
    inputs.velocityRad = velocity.getValue().in(RadiansPerSecond) / WristConstants.GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    wristTalon.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setPositionTarget(Rotation2d angle) {
    wristTalon.setControl(
        request.withPosition(
            angle.minus(Rotation2d.kZero).getRotations() * WristConstants.GEAR_RATIO));
  }
}
