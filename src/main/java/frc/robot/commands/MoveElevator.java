package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevator extends Command {
    private Elevator elevator;
    private double height;

    public MoveElevator(Elevator elevator, double height) {
        this.elevator = elevator;
        this.height = height;
    }

    @Override
    public void initialize() {
        elevator.runSetpoint(height);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
