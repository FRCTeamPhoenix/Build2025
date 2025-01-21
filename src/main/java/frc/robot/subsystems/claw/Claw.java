package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private ClawIO io;
    private ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    public Claw(ClawIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void runForward() {
        io.setVoltage(3);
    }

    public void runReverse() {
        io.setVoltage(-3);
    }

    public void stop() {
        io.setVoltage(0);
    }
}
