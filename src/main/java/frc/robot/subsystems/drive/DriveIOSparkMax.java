package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.util.Gyro;

public class DriveIOSparkMax implements DriveIO {
  private static final double GEAR_RATIO = 6.0;

  private final CANSparkMax leftLeader;
  private final CANSparkMax rightLeader;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final Gyro gyro;

  public DriveIOSparkMax() {
    leftLeader = new CANSparkMax(1, MotorType.kBrushless);
    rightLeader = new CANSparkMax(2, MotorType.kBrushless);
    leftFollower = new CANSparkMax(3, MotorType.kBrushless);
    rightFollower = new CANSparkMax(4, MotorType.kBrushless);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

    gyro = Gyro.getInstance();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        rightEncoder.getVelocity() / GEAR_RATIO);
    inputs.gyroYawRad = gyro.getYawAngle();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }
}
