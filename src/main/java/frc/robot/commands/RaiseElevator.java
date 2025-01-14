package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class RaiseElevator extends Command {
    private Elevator elevator;

    public RaiseElevator(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.runSetpoint(ElevatorConstants.maxHeight - ElevatorConstants.minHeight);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
