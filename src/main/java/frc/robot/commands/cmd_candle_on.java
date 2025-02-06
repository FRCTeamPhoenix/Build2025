package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.candle.Candle;

public class cmd_candle_on extends Command {
    private final Candle candle;

    public cmd_candle_on(Candle candle) {
        this.candle = candle;
    }

    @Override
    public void initialize() {
        candle.setColor(255, 255, 255);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
