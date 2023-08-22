package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    private final ClawIO io;

    public Claw(ClawIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
        io.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Claw", inputs);

    }

    public void setSpeed(double motorVolts) {
        io.setSpeed(motorVolts);
    }

    public CommandBase grab() {
        return new FunctionalCommand(
            () -> io.setSensor(true),
            () -> io.setSpeed(12.0),
            (interrupted) -> io.setSpeed(0.0),
            () -> io.getSensor(),
            this
        );
    }

    public CommandBase release() {
        return new FunctionalCommand(
            () -> io.setSensor(false),
            () -> io.setSpeed(-12.0),
            (interrupted) -> io.setSpeed(0.0),
            () -> !io.getSensor(),
            this
        );
    }

    public boolean open() {
        return !io.getSensor();
    }

}
