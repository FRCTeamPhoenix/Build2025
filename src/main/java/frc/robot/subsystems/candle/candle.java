package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

public class Candle extends SubsystemBase {
    private final CANdle candle; 
    private final CANdleConfiguration config;

    private FireAnimation fireAnimation = new FireAnimation(1, 0.25, 300, 1, 0 );
    private RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 0.5, 300);
    private RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(1, 0.25, 300 );



    /**
     * Creates a new CANdle object with the supplied CANID
     * @param canID CANID for CANdle device
     */
    public Candle(int canID) {
        this.candle = new CANdle(canID);
        this.config = new CANdleConfiguration();
        candle.configAllSettings(config);
    }

    /**
     * Sets the LED color with the supplied RGB color
     * @param red Amount of red on the RGB scale [0, 255]
     * @param green Amount of green on the RGB scale [0, 255]
     * @param blue Amount of blue on the RGB scale [0, 255]
     */
    public void setColor(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    /**
     * Blinks the lights on and off three times with the supplied RGB color
     * @param durationSeconds Duration in seconds between on -> off and off -> on
     * @param red Amount of red on the RGB scale [0, 255]
     * @param green Amount of green on the RGB scale [0, 255]
     * @param blue Amount of blue on the RGB scale [0, 255]
     */
    public void blinkLights(double durationSeconds, int red, int green, int blue) {
        for (int i = 0; i < 3; i++) {
            setColor(red, green, blue);
            Timer.delay(durationSeconds);
            setColor(CANdleConstants.COLOR_OFF, CANdleConstants.COLOR_OFF, CANdleConstants.COLOR_OFF);
            Timer.delay(durationSeconds);
        }
    }

    /**
     * Takes in an animation (probably something different than the presets) and makes candle to do it 
     * @param animation Animation that candle will do 
     */
    public void animate(Animation animation) {
        candle.animate(animation);
    }

    /**
     * Updates last error from candle 
     */
    public ErrorCode getLastError() {
        return candle.getLastError();
    }

    /**
     * Updates faults from candle 
     */
    public ErrorCode getFaults(CANdleFaults faults) {
        return candle.getFaults(faults);
    }

    
    /**
     * Uses preset fire animation and makes candle do it
     */
    public void fireAnimate() {
        candle.animate(fireAnimation);
    }

    /**
     * Uses preset rainbow animation and makes candle do it
     */
    public void rainbowAnimate() {
        candle.animate(rainbowAnimation);
    }

    /**
     * Uses preset rgbFade animation and makes candle do it
     */
    public void rgbFade() {
        candle.animate(rgbFadeAnimation);

    }
}
