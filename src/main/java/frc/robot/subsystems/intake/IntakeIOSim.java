package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants.Intake.IntakeSimConstants;

public class IntakeIOSim implements IntakeIO {
    // from here
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/.java
    // The P gain for the PID controller that drives this arm.

    private final PWMSparkMax motor = new PWMSparkMax(IntakeSimConstants.kMotorPort);
    private final Encoder encoder = new Encoder(IntakeSimConstants.kEncoderPorts[0], IntakeSimConstants.kEncoderPorts[1]);
    private final EncoderSim m_encoderSim = new EncoderSim(encoder);

    public IntakeIOSim() {
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angle = encoder.getDistance();
        inputs.angleRadsPerSec = encoder.getRate();
        inputs.currentAmps = new double[] { 0 };

    }

    @Override
    public void setVoltage(double motorVolts) {
        motor.set(motorVolts / RobotController.getBatteryVoltage());
        m_encoderSim.setRate(motorVolts / RobotController.getBatteryVoltage());
    }
}
