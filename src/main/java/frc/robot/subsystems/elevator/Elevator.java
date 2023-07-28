package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Elevator.ElevatorPhysicalConstants.ELEVATOR_PID;

public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public enum State {
        MANUAL,
        PID
    }

    private LoggedDashboardNumber p = new LoggedDashboardNumber("Elevator/P", ELEVATOR_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("Elevator/I", ELEVATOR_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("Elevator/D", ELEVATOR_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("Elevator/FF", ELEVATOR_PID[3]);


    private State state = State.MANUAL;
    private double setpoint = 0;

    private final ElevatorIO io;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d = 
    m_mech2dRoot.append(new MechanismLigament2d("Elevator", 36, 36));

    public Elevator(ElevatorIO io) {
        this.io = io;
        SmartDashboard.putData("Elevator Mech", m_mech2d);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        m_elevatorMech2d.setLength(io.getDistance());

        Logger.getInstance().processInputs("Elevator", inputs);
    }

    public void setVoltage(double motorVolts) {
        io.setVoltage(motorVolts);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(io.getDistance() - setpoint) < ELEVATOR_TOLERANCE;
    }
    
}
