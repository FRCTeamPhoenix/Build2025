package frc.robot.subsystems.elevator;

import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase {
    
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward feedforward;
    private Double setpoint = 0.0;

    //Mechanism2D
    private final LoggedMechanism2d mech = new LoggedMechanism2d(24, 24);
    private final LoggedMechanismRoot2d root = mech.getRoot("root", 12, 0);
    private final LoggedMechanismLigament2d elevator = root.append(new LoggedMechanismLigament2d("elevator", ElevatorConstants.MIN_HEIGHT, 90, 10, new Color8Bit(135, 140, 148)));


    public Elevator(ElevatorIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
            pidController = new ProfiledPIDController(11, 1, 0.0, 
            new TrapezoidProfile.Constraints(1.5, 0.5)); 
            feedforward = new ElevatorFeedforward(0.316, 0.565, 0.506);               
                break;
            case SIM:
                pidController = new ProfiledPIDController(0.42, 0.730, 0.50, 
                        new TrapezoidProfile.Constraints(3, 3));
                feedforward = new ElevatorFeedforward(0, 0.4, 0);               
                break;
            default:
                pidController = new ProfiledPIDController(0.0, 0.0, 0.0, 
                        new TrapezoidProfile.Constraints(3, 3));
                feedforward = new
                 ElevatorFeedforward(0, 0, 0);               
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Mech2D", mech);

        if (setpoint != null) {Logger.recordOutput("Elevator/Setpoint", setpoint);}
        else {Logger.recordOutput("Elevator/Setpoint", -1.0);}

        elevator.setLength(inputs.heightMeters + ElevatorConstants.MIN_HEIGHT);

        if (setpoint != null) {
            pidController.setGoal(setpoint);

            io.setVoltage(pidController.calculate(inputs.heightMeters) + feedforward.calculate(pidController.getSetpoint().velocity));
        }

        SmartDashboard.putData("Elevator PID", pidController);
    }

    public void goToPosition(IntSupplier positionIndex) {
        this.setpoint = ElevatorConstants.POSITIONS[positionIndex.getAsInt()];
    }

    public void runSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getHeight() {
        return inputs.heightMeters;
    }

    public double getVelocity() {
        return inputs.velocityMetersPerSec;
    }

    public void stop() {
        io.setVoltage(feedforward.calculate(0));
        setpoint = null;
    }

    public void runCharacterization(double volts) {
        setpoint = null;
        io.setVoltage(volts);
    }

    public double getFFCharacterizationVelocity() {
        return inputs.velocityRotationsPerSec;
    }

    public boolean atSetpoint() {
        return pidController.atGoal();
    }
}
