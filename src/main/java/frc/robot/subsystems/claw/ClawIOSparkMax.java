package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.Claw.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;
import frc.robot.Constants.ElectricalLayout;

public class ClawIOSparkMax implements ClawIO {
    private CANSparkMax clawMotor;
    private RelativeEncoder clawEncoder;
    private DigitalInput gamePieceButton = new DigitalInput(ElectricalLayout.CLAW_PIECE_BUTTON);

    public ClawIOSparkMax() {
        clawMotor = new CANSparkMax(ElectricalLayout.CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
        clawMotor.restoreFactoryDefaults();
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        clawMotor.setInverted(true);

        clawEncoder = clawMotor.getEncoder();
        clawEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        clawEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60);
        
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.speed = clawMotor.get();
        inputs.encoderPosition = clawEncoder.getPosition();
        inputs.appliedVolts = clawMotor.getAppliedOutput() * clawMotor.getBusVoltage();
        inputs.currentAmps = new double[] {clawMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {clawMotor.getMotorTemperature()};
        inputs.gamePiece = gamePieceButton.get();
    }

    @Override
    public void setSpeed(double speed) {
        clawMotor.set(speed);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        clawMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean getSensor() {
        return gamePieceButton.get();
    }
}
