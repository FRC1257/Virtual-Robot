package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants.Claw.ClawSimConstants;

public class ClawIOSim implements ClawIO {
    private final PWMSparkMax motor = new PWMSparkMax(0);
    private final Encoder m_encoder =
      new Encoder(ClawSimConstants.kEncoderAChannel, ClawSimConstants.kEncoderBChannel);
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    private boolean gamePiece = false;
    
    public ClawIOSim() {
        m_encoder.setDistancePerPulse(ClawSimConstants.kEncoderDistancePerPulse);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.speed = motor.get();
        inputs.encoderPosition = m_encoderSim.getDistance();
        inputs.appliedVolts = motor.get() * 12;
        inputs.currentAmps = new double[] { 0 };
        inputs.tempCelsius = new double[] { 0 };
        inputs.gamePiece = gamePiece;
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
        m_encoderSim.setRate(speed * ClawSimConstants.kEncoderDistancePerPulse);
    }

    @Override
    public boolean getSensor() {
        return gamePiece;
    }

    @Override
    public void setSensor(boolean gamePiece) {
        this.gamePiece = gamePiece;
    }
}
