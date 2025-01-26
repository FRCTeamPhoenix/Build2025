package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.util.SwerveUtils.PhoenixFF;

public class Claw extends SubsystemBase {

    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
    private final PIDController controller;
    private final PhoenixFF feedforward;
    private Double setpoint = null;

    public Claw(ClawIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL:
                controller = new PIDController(-1.0, -1.0, -1.0);
                feedforward = new PhoenixFF(-1, -1);
                break;
            case SIM:
                controller = new PIDController(0.002, 0.1, 0.000002);
                feedforward = new PhoenixFF(0.0, 0.0075);
                break;
            default:
                controller = new PIDController(0, 0, 0);
                feedforward = new PhoenixFF(0, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);

        if (setpoint != null) {
            double rotationsPerSec = setpoint / ClawConstants.INNER_WHEEL_RADIUS;
            Logger.recordOutput("Claw/SetpointRotations", rotationsPerSec);
            io.setVoltage(controller.calculate(inputs.velocityRotationsPerSec, rotationsPerSec) + feedforward.calculate(rotationsPerSec));
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
