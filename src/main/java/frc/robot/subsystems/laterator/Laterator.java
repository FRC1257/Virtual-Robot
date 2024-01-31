package frc.robot.subsystems.laterator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static frc.robot.Constants.Laterator.*;
import static frc.robot.Constants.Laterator.LateratorPhysicalConstants.LATERATOR_PID;

public class Laterator extends SubsystemBase {
    private final LateratorIOInputsAutoLogged inputs = new LateratorIOInputsAutoLogged();

    private LoggedDashboardNumber p = new LoggedDashboardNumber("Laterator/P", LATERATOR_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("Laterator/I", LATERATOR_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("Laterator/D", LATERATOR_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("Laterator/FF", LATERATOR_PID[3]);

    private double setpoint = 0;

    private final LateratorIO io;

    // Create a Mechanism2d visualization of the Laterator
    private MechanismLigament2d LateratorMechanism;

    public Laterator(LateratorIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    public double highSetpoint() {
        return io.LATERATOR_MAX_HEIGHT;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Laterator", inputs);

        LateratorMechanism.setLength(io.getDistance());

        // Update the PID constants if they have changed
        if (p.get() != io.getP()) 
            io.setP(p.get());
        
        if (i.get() != io.getI())
            io.setI(i.get());
        
        if (d.get() != io.getD())
            io.setD(d.get());
        
        if (ff.get() != io.getFF())
            io.setFF(ff.get());
        
        // Log Inputs
        Logger.getInstance().processInputs("Laterator", inputs);
    }

    public void setVoltage(double motorVolts) {
        // limit the Laterator if its past its limit
        if (io.getDistance() > io.LATERATOR_MAX_HEIGHT && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getDistance() < io.LATERATOR_MIN_HEIGHT && motorVolts < 0) {
            motorVolts = 0;
        }

        io.setVoltage(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed * 12);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(io.getDistance() - setpoint) < LATERATOR_TOLERANCE;
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        LateratorMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return LateratorMechanism.append(mechanism);
    }

    public MechanismLigament2d getLateratorMechanism() {
        return new MechanismLigament2d("Laterator", 5, 0, 5, new Color8Bit(Color.kOrange));
    }

    public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }
    
}
