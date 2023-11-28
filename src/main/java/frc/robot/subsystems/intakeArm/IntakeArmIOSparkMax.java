package frc.robot.subsystems.intakeArm;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.IntakeArm;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.IntakeArm.IntakeArmPhysicalConstants.*;

public class IntakeArmIOSparkMax implements IntakeArmIO {
    // Motor and Encoders
    private CANSparkMax intakeArmMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    private double setpoint = 0;

    public IntakeArmIOSparkMax() {
        intakeArmMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        intakeArmMotor.restoreFactoryDefaults();
        intakeArmMotor.setIdleMode(IdleMode.kBrake);
        intakeArmMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pidController = intakeArmMotor.getPIDController();
        pidController.setOutputRange(-1, 1);

        encoder = intakeArmMotor.getEncoder();
        encoder.setPositionConversionFactor(IntakeArm.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(IntakeArm.POSITION_CONVERSION_FACTOR / 60);
        encoder.setPosition(0.6);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);
    }

    @Override
    public void setPIDConstants(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
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

    /** Returns the current distance measurement. */
    @Override
    public double getAngle() {
        return encoder.getPosition();
    }

    /** Go to Setpoint */
    @Override
    public void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setBrake(boolean brake) {
        intakeArmMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(encoder.getPosition() - setpoint) < IntakeArm.INTAKE_ARM_PID_TOLERANCE;
    }

    @Override
    public void setP(double p) {
        pidController.setP(p);
    }

    @Override
    public void setI(double i) {
        pidController.setI(i);
    }

    @Override
    public void setD(double d) {
        pidController.setD(d);
    }

    @Override
    public void setFF(double ff) {
        pidController.setFF(ff);
    }

    @Override
    public double getP() {
        return pidController.getP();
    }

    @Override
    public double getI() {
        return pidController.getI();
    }

    @Override
    public double getD() {
        return pidController.getD();
    }

    @Override
    public double getFF() {
        return pidController.getFF();
    }

}
