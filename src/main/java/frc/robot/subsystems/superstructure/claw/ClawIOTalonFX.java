package frc.robot.subsystems.superstructure.claw;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClawConstants;

public class ClawIOTalonFX implements ClawIO {

  private TalonFX clawTalon = new TalonFX(CANConstants.CLAW_ID);
  private LaserCan laserCan = new LaserCan(CANConstants.LASERCAN_ID);
  private final Alert motorAlert = new Alert("Claw motor is disconnected", AlertType.kError);
  private final Alert sensorAlert = new Alert("Claw LaserCAN is disconnected", AlertType.kError);

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final boolean isInverted = false;

  public ClawIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    clawTalon.getConfigurator().apply(config);
    setBrakeMode(true);

    try {
      sensorAlert.set(false);
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (Exception e) {
      sensorAlert.set(true);
      System.err.println("Configuration failed: " + e.getMessage());
    }

    appliedVolts = clawTalon.getMotorVoltage();
    current = clawTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, current);
    clawTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(appliedVolts, current);

    inputs.connected = status.isOK();
    motorAlert.set(!inputs.connected);

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    try {
      sensorAlert.set(false);
      inputs.intakeSensor =
          laserCan.getMeasurement().distance_mm < ClawConstants.LASERCAN_TRIGGER_DISTANCE;
    } catch (Exception e) {
      sensorAlert.set(true);
      inputs.intakeSensor = false;
      System.err.println("Failed to read LaserCAN: " + e);
    }
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
