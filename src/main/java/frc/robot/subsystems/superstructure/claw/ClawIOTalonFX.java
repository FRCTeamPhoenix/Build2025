package frc.robot.subsystems.superstructure.claw;

import au.grapplerobotics.ConfigurationFailedException;
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
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClawConstants;
import org.littletonrobotics.junction.Logger;

public class ClawIOTalonFX implements ClawIO {

  private TalonFX clawTalon = new TalonFX(CANConstants.CLAW_ID);
  private LaserCan laserCan = new LaserCan(CANConstants.LASERCAN_ID);

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
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
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
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    try {
      inputs.intakeSensor =
          laserCan.getMeasurement().distance_mm < ClawConstants.LASERCAN_TRIGGER_DISTANCE;
      Logger.recordOutput("Lasercan", laserCan.getMeasurement().distance_mm);
    } catch (Exception e) {
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
