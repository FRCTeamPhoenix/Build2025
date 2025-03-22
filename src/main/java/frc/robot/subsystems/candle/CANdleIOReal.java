package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.CANConstants;

public class CANdleIOReal implements CANdleIO {
  private CANdle candle = new CANdle(CANConstants.LEFT_CANDLE_ID);

  private CANdleState state = CANdleState.Off;

  private FireAnimation fireAnimation = new FireAnimation(1, 0.01, 37, 1, 0);
  private RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 0.01, 37);
  private RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(1, 0.01, 37);
  private LarsonAnimation larsonAnimation =
      new LarsonAnimation(255, 30, 0, 0, 0.01, 37, BounceMode.Front, 5, 0);

  public CANdleIOReal() {
    candle.configAllSettings(new CANdleConfiguration());
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      setMode(CANdleState.Red);
    } else {
      setMode(CANdleState.Blue);
    }
  }
  // Set the RGBs for Phoenix Colors once Business Art gets them for us
  @Override
  public void setMode(CANdleState mode) {
    if (state == mode) {
      return;
    }
    candle.clearAnimation(0);
    state = mode;
    switch (mode) {
      case FireAnimation:
        candle.animate(fireAnimation);
        break;
      case RainbowAnimation:
        candle.animate(rainbowAnimation);
        break;
      case RgbFadeAnimation:
        candle.animate(rgbFadeAnimation);
        break;
      case LarsonAnimation:
        candle.animate(larsonAnimation);
        break;
      case Red:
        candle.setLEDs(255, 0, 0);
        break;
      case Blue:
        candle.setLEDs(0, 0, 255);
        break;
      case Green:
        candle.setLEDs(0, 255, 0);
        break;
      case Orange:
        candle.setLEDs(255, 30, 0);
        break;
      case Yellow:
        candle.setLEDs(255, 69, 0);
        break;
      case Cyan:
        candle.setLEDs(0, 255, 255);
        break;
      default:
        candle.setLEDs(0, 0, 0);
        break;
    }
  }

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.mode = state;
    inputs.busVolts = new double[] {candle.getBusVoltage()};
    inputs.railVolts = new double[] {candle.get5VRailVoltage()};
  }
}
