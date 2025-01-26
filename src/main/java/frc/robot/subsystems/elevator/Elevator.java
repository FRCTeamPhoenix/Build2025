package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController pidController;
    private final double kG;
    private Double setpoint = 0.0;

    //Mechanism2D
    private final LoggedMechanism2d mech = new LoggedMechanism2d(24, 24);
    private final LoggedMechanismRoot2d root = mech.getRoot("root", 12, 0);
    private final LoggedMechanismLigament2d elevator = root.append(new LoggedMechanismLigament2d("elevator", ElevatorConstants.MIN_HEIGHT, 90, 10, new Color8Bit(135, 140, 148)));


    public Elevator(ElevatorIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                pidController = new ProfiledPIDController(-1.0, -1.0, -1.0, 
                        new TrapezoidProfile.Constraints(3, 3)); 
                kG = 0.0;               
                break;
            case SIM:
                pidController = new ProfiledPIDController(0.42, 0.730, 0.50, 
                        new TrapezoidProfile.Constraints(3, 3));
                kG = 0.4;
                break;
            default:
                pidController = new ProfiledPIDController(0.0, 0.0, 0.0, 
                        new TrapezoidProfile.Constraints(3, 3));
                kG = 0.0;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Mech2D", mech);

        if (setpoint != null) {Logger.recordOutput("Elevator/Setpoint", setpoint);}
        else {Logger.recordOutput("Elevator/Setpoint", -1);}

        elevator.setLength(inputs.height + ElevatorConstants.MIN_HEIGHT);

        if (setpoint != null) {
            pidController.setGoal(setpoint);

            io.setVoltage(pidController.calculate(inputs.height) + kG);
        }
    }

    public void goToPosition(int positionIndex) {
        this.setpoint = ElevatorConstants.POSITIONS[positionIndex];
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

    public boolean atSetpoint() {
        boolean returnBool = false;
        if (setpoint == inputs.height){
            returnBool = true;
        }
        return returnBool;
    }

}
