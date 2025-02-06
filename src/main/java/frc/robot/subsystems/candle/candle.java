package frc.robot.subsystems.candle;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

public class candle extends SubsystemBase{
    private CANdle candle; 

    public candle(int canID) {
        CANdle candle = new CANdle(canID);
        CANdleConfiguration config = new CANdleConfiguration();
        candle.configAllSettings(config);
    }

    public void setColor(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    public void blinkLights(double durationSeconds, int red, int green, int blue) {
        for (int i = 0; i < 3; i++) {
            setColor(red, green, blue);
            Timer.delay(durationSeconds);
            setColor(CANdleConstants.COLOR_OFF, CANdleConstants.COLOR_OFF, CANdleConstants.COLOR_OFF);
            Timer.delay(durationSeconds);
        }
    }
}
