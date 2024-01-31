package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.Intake;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

public class IntakeIOSparkMax implements IntakeIO {
    // Motor and Encoders
    private CANSparkMax intakeArmMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;

    public IntakeIOSparkMax() {
        intakeArmMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeArmMotor.restoreFactoryDefaults();
        intakeArmMotor.setIdleMode(IdleMode.kBrake);
        intakeArmMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pidController = intakeArmMotor.getPIDController();
        pidController.setOutputRange(-1, 1);

        encoder = intakeArmMotor.getEncoder();
        encoder.setPositionConversionFactor(Intake.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(Intake.POSITION_CONVERSION_FACTOR / 60);
        encoder.setPosition(0.6);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angle = encoder.getPosition();
        inputs.angleRadsPerSec = encoder.getVelocity();
        inputs.appliedVolts = intakeArmMotor.getAppliedOutput() * intakeArmMotor.getBusVoltage();
        inputs.currentAmps = new double[] {intakeArmMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {intakeArmMotor.getMotorTemperature()};
    }

    /** Run open loop at the specified voltage. */
    @Override
    public void setVoltage(double motorVolts) {
        intakeArmMotor.setVoltage(motorVolts);
    }

    @Override
    public void setBrake(boolean brake) {
        intakeArmMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

}
