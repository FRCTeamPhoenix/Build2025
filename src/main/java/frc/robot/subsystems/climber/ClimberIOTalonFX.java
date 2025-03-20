package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

public class ClimberIOTalonFX implements ClimberIO {

  private TalonFX climberTalon = new TalonFX(CANConstants.CLIMBER_ID);
  private final Alert motorAlert = new Alert("Climber motor is disconnected", AlertType.kError);

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

    appliedVolts = climberTalon.getMotorVoltage();
    current = climberTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, current);
    climberTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(appliedVolts, current);
    motorAlert.set(!status.isOK());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    climberTalon.setControl(new VoltageOut(voltage));
  }
}
