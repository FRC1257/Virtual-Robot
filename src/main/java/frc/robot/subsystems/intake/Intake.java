package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }

    public void setVoltage(double motorVolts) {
        io.setVoltage(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed * 12);
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }
}
