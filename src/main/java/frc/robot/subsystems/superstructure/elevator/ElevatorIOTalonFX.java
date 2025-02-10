package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
  private TalonFX followerTalon = new TalonFX(CANConstants.ELEVATOR_FOLLOWER_ID);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  private final boolean isInverted = false;
  private final boolean brakeEnabled = true;

  public ElevatorIOTalonFX() {
    followerTalon.setControl(new Follower(elevatorTalon.getDeviceID(), false));

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    elevatorTalon.getConfigurator().apply(config);

    elevatorTalon.setPosition(0);

    position = elevatorTalon.getPosition();
    velocity = elevatorTalon.getVelocity();
    appliedVolts = elevatorTalon.getMotorVoltage();
    current = elevatorTalon.getSupplyCurrent();

    followerAppliedVolts = followerTalon.getMotorVoltage();
    followerCurrent = followerTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current, followerAppliedVolts, followerCurrent);
    elevatorTalon.optimizeBusUtilization();
    followerTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVolts, current, followerAppliedVolts, followerCurrent);

    inputs.heightMeters =
        position.getValueAsDouble()
            / ElevatorConstants.GEAR_RATIO
            * (2 * Math.PI * ElevatorConstants.MAGIC_NUMBER);
    inputs.velocityMetersPerSec =
        velocity.getValueAsDouble()
            / ElevatorConstants.GEAR_RATIO
            * (2 * Math.PI * ElevatorConstants.MAGIC_NUMBER);
    inputs.velocityRotationsPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts =
        new double[] {appliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
    inputs.currentAmps =
        new double[] {current.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorTalon.setControl(new VoltageOut(voltage));
  }
}
