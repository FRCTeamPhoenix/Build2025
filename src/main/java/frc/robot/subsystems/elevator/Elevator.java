package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pidController;
    private final double kG;
    private Double setpoint = 0.0;

    //Mechanism2D
    private final LoggedMechanism2d mech = new LoggedMechanism2d(24, 24);
    private final LoggedMechanismRoot2d root = mech.getRoot("root", 12, 0);
    private final LoggedMechanismLigament2d elevator = root.append(new LoggedMechanismLigament2d("elevator", ElevatorConstants.minHeight, 90, 10, new Color8Bit(135, 140, 148)));


    public Elevator(ElevatorIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                pidController = new PIDController(-1.0, -1.0, -1.0); 
                kG = 0.0;               
                break;
            case SIM:
                pidController = new PIDController(0.42, 0.730, 0.50);
                kG = 0.4;
                break;
            default:
                pidController = new PIDController(0.0, 0.0, 0.0);
                kG = 0.0;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (setpoint != null) {
            Logger.recordOutput("Elevator/Setpoint", setpoint);
        }
        else {
            Logger.recordOutput("Elevator/Setpoint", -1);
        }

        Logger.recordOutput("Elevator/Mech2D", mech);

        elevator.setLength(inputs.height + ElevatorConstants.minHeight);

        if (setpoint != null) {
            //positionController.setGoal(setpoint);

            io.setVoltage(pidController.calculate(inputs.height, setpoint) + kG);
        }
    }

    public void runSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void stop() {
        io.setVoltage(0.0);
        setpoint = null;
    }
}
