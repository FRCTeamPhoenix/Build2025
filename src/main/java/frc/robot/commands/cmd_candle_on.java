package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.candle.candle;

public class cmd_candle_on extends Command {
    private candle m_candle;

    public cmd_candle_on(candle m_candle) {
        this.m_candle = m_candle;
    }

    @Override
    public void initialize() {
        m_candle.setColor(255, 255, 255);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
