package frc.robot.subsystems.candle;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

public class Candle extends SubsystemBase{
    private final CANdle candle; 
    private final CANdleConfiguration config;

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
}
