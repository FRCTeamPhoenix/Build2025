package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.candle.Candle;

public class cmd_candle_blink extends Command {
    private final Candle m_candle;

    public cmd_candle_blink(Candle CANdle) {
        m_candle = CANdle;
      }

      @Override
      public void initialize() {}
    
      @Override
      public void execute() {
        m_candle.blinkLights(1.5, 255, 255, 255); // TODO: finalize off time and on time
      }
    
      @Override
      public void end(boolean interrupted) {
        m_candle.blinkLights(0, 0, 0, 0);
      }
    
      @Override
      public boolean isFinished() {
        return false;
      }
}