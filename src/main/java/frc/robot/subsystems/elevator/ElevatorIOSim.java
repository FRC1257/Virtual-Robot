package frc.robot.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import static frc.robot.Constants.Elevator.ElevatorSimConstants.*;

public class ElevatorIOSim implements ElevatorIO {
    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(
            kElevatorKp,
            kElevatorKi,
            kElevatorKd,
            new TrapezoidProfile.Constraints(2.45, 2.45)
        );
    
    ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            kElevatorkS,
            kElevatorkG,
            kElevatorkV,
            kElevatorkA
        );

    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

    private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

    private final ElevatorSim sim =
        new ElevatorSim(
            m_elevatorGearbox,
            kElevatorGearing,
            kCarriageMass,
            kElevatorDrumRadius,
            kMinElevatorHeightMeters,
            kMaxElevatorHeightMeters,
            true,
            VecBuilder.fill(0.01)
        );

    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    
    public ElevatorIOSim() {
        m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);
        m_encoderSim.setDistance(sim.getPositionMeters());
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMeters = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double motorVolts) {
        sim.setInputVoltage(motorVolts);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        m_controller.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(m_encoder.getDistance());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput);
    }

    @Override
    public double getDistance() {
        return sim.getPositionMeters();
    }

    @Override
    public boolean atSetpoint() {
        return m_controller.atGoal();
    }
}
