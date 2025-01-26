package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
    private final PIDController controller;
    private Double setpoint = null;

    public Claw(ClawIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                controller = new PIDController(-1.0, -1.0, -1.0);
                break;
            case SIM:
                controller = new PIDController(0.05, 0, 0);
                break;
            default:
                controller = new PIDController(0, 0, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);

        if (setpoint != null) {
            Logger.recordOutput("Claw/Setpoint", setpoint);
            io.setVoltage(controller.calculate(inputs.velocityMetersPerSec, setpoint));
        }
        else {
            io.setVoltage(0);
        }
    }

    public void runForward() {
        this.setpoint = 3.0;
    }

    public void runReverse() {
        this.setpoint = -3.0;
    }

    public void stop() {
        io.setVoltage(0);
    }

    public void holdPosition() {
        this.setpoint = 0.0;
    }
}
