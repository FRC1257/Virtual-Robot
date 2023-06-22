package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Drivetrain.*;
import frc.robot.util.Gyro;

public class Drive extends SubsystemBase {
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

  private final Field2d m_field = new Field2d();

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

  private RamseteController ramseteController = new RamseteController(DRIVE_TRAJ_RAMSETE_B, DRIVE_TRAJ_RAMSETE_ZETA);
  private Trajectory trajectory;
  private Timer pathTimer;

  private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH_M);
  private final DifferentialDrivePoseEstimator poseEstimator;

  public enum States {
    MANUAL,
    TRAJECTORY
  }

  private States defaultState = States.MANUAL;
  private States state = States.MANUAL;

  /** Creates a new Drive. */
  public Drive(DriveIO io, Pose2d initialPoseMeters) {
    this.io = io;
    SmartDashboard.putData("Field", m_field);
    poseEstimator = new DifferentialDrivePoseEstimator(driveKinematics, Rotation2d.fromDegrees(-Gyro.getInstance().getRobotAngle()), io.getLeftPositionMeters(), io.getRightPositionMeters(), initialPoseMeters);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry and log the new pose
    odometry.update(new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometry", getPose());

    m_field.setRobotPose(getPose());
  }

  /** Run open loop at the specified percentage. */
  public void drivePercent(double leftPercent, double rightPercent) {
    io.setVoltage(leftPercent * 12.0, rightPercent * 12.0);
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

  public void followTrajectory() {
    if(trajectory == null) {
      state = defaultState;
      return;
    }

    if(pathTimer.get() > trajectory.getTotalTimeSeconds()) {
      pathTimer.stop();
      pathTimer.reset();
      trajectory = null;

      // stop the bot
      stop();
      state = defaultState;
      return;
    }

    Trajectory.State currentState = trajectory.sample(pathTimer.get());
    
    ChassisSpeeds chassisSpeeds = ramseteController.calculate(poseEstimator.getEstimatedPosition(), currentState); 
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

    io.setVelocity(wheelSpeeds);
  }

  public void driveTrajectory(Trajectory trajectory) {
    zero();
    setRobotPose(trajectory.getInitialPose());

    this.trajectory = trajectory;

    m_field.getObject("traj").setTrajectory(trajectory);       

    pathTimer.reset();
    pathTimer.start();

    state = States.TRAJECTORY;
  }

  private void setRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(-Gyro.getInstance().getRobotAngle()), io.getLeftPositionMeters(), io.getRightPositionMeters(), pose);
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  public void zero() {
    Gyro.getInstance().zeroAll();
    io.zero();
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the position of the left wheels in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the position of the right wheels in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }
}
