// Copyright
// Right candle has been disabled until we install it

package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import frc.robot.Constants.CANConstants;

public class CANdleIOReal implements CANdleIO {
  private CANdle leftCandle = new CANdle(CANConstants.LEFT_CANDLE_ID);
  // private CANdle rightCandle = new CANdle(CANConstants.RIGHT_CANDLE_ID);

  private FireAnimation fireAnimation = new FireAnimation(1, 0.25, 300, 1, 0);
  private RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 0.5, 300);
  private RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(1, 0.25, 300);

  public CANdleIOReal() {
    leftCandle.configAllSettings(new CANdleConfiguration());
    // rightCandle.configAllSettings(new CANdleConfiguration());
  }
  // Set the RGBs for Phoenix Colors once Business Art gets them for us
  @Override
  public void setMode(CANdleState mode) {
    switch (mode) {
      case FireAnimation:
        leftCandle.animate(fireAnimation);
        // rightCandle.animate(fireAnimation);
        break;
      case RainbowAnimation:
        leftCandle.animate(rainbowAnimation);
        // rightCandle.animate(rainbowAnimation);
        break;
      case RgbFadeAnimation:
        leftCandle.animate(rgbFadeAnimation);
        //  rightCandle.animate(rgbFadeAnimation);
        break;
      case Red:
        leftCandle.setLEDs(255, 0, 0);
        // rightCandle.setLEDs(255, 0, 0);
        break;
      case Blue:
        leftCandle.setLEDs(0, 0, 255);
        //  rightCandle.setLEDs(0, 0, 255);
        break;
      case Green:
        leftCandle.setLEDs(0, 255, 0);
        // rightCandle.setLEDs(0, 255, 0);
        break;
      case Orange:
        leftCandle.setLEDs(255, 69, 0);
        // rightCandle.setLEDs(255, 69, 0);
        break;
      case PhoenixOrange: // Phoenix colors have been commented out until we actually get them
        leftCandle.setLEDs(224, 79, 13);
        // rightCandle.setLEDs(224, 79, 13);
      case PhoenixRed:
        leftCandle.setLEDs(217, 83, 79);
        // rightCandle.setLEDs(217, 83, 79);
        break;
      //case PhoenixYellow:
        //leftCandle.setLEDs(255, 69, 0);
        // rightCandle.setLEDs(255, 69, 0);
        //break;
      default:
        leftCandle.setLEDs(0, 0, 0);
        // rightCandle.setLEDs(0, 0, 0);
        break;
    }
  }
}
// Peter Halassa is an uwu furry femboy
